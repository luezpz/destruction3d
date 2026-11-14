// Static bond-graph solver (PLAN §2): owns buffers and drives the kernels. Supports are eliminated; K u = f by
// block-Jacobi PCG, warm-started, time-sliced. Setup allocates; per-solve calls do not.

import { ChunkFlag, ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { BOND_K_STRIDE, BondViews, computeBondForces, computeBondStiffness } from '../kernels/bondStiffness';
import { BlockCsr, assembleBlockCsr, buildBlockCsr } from '../kernels/blockCsr';
import { PCG_ITERS, PcgWork, createPcgWork, invertDiagBlocks, pcgBegin, pcgIterate } from '../kernels/pcg';

/** Builds the kernel view bundle from a model and material table (E in GPa, nu per material id). */
export function bondViews(model: ChunkModel, table: MaterialTable): BondViews {
  return {
    chunkPos: model.chunkPos, bondA: model.bondA, bondB: model.bondB, bondArea: model.bondArea,
    bondCentroid: model.bondCentroid, bondNormal: model.bondNormal, bondT1: model.bondT1, bondT2: model.bondT2,
    bondI1: model.bondI1, bondI2: model.bondI2, bondJ: model.bondJ, bondMaterial: model.bondMaterial,
    matE: Float64Array.from(table.materials, (m) => m.E), matNu: Float64Array.from(table.materials, (m) => m.nu),
  };
}

export class BondSolver {
  readonly n: number;
  readonly m: number;
  readonly views: BondViews;
  /** Free row of each chunk, -1 for supports: (n,). */
  readonly freeIdx: Int32Array;
  readonly csr: BlockCsr;
  /** Per-bond damage d (m,) and alive flag (m,); call updateStiffness() after changing them. */
  readonly damage: Float64Array;
  readonly alive: Uint8Array;
  /** Per chunk (n,): 0 once detached from the static structure (handed to Rapier); excluded from loads. */
  readonly active: Uint8Array;
  /** Local bond stiffness kd (6m,), world bond blocks (108m,). */
  readonly kd: Float64Array;
  readonly bondK: Float64Array;
  readonly work: PcgWork;
  /** Load in free numbering (6 nf,). */
  private readonly fFree: Float64Array;

  /** @param model chunk graph; chunks flagged SUPPORT are fixed. */
  constructor(model: ChunkModel, table: MaterialTable) {
    this.n = model.chunkCount;
    this.m = model.bondCount;
    this.views = bondViews(model, table);
    this.freeIdx = new Int32Array(this.n);
    let nf = 0;
    for (let i = 0; i < this.n; i++) this.freeIdx[i] = model.chunkFlags[i] & ChunkFlag.SUPPORT ? -1 : nf++;
    this.csr = buildBlockCsr(nf, this.freeIdx, model.bondA, model.bondB, this.m);
    this.damage = new Float64Array(this.m);
    this.alive = new Uint8Array(this.m).fill(1);
    this.active = new Uint8Array(this.n).fill(1);
    this.kd = new Float64Array(6 * this.m);
    this.bondK = new Float64Array(BOND_K_STRIDE * this.m);
    this.work = createPcgWork(nf);
    this.fFree = new Float64Array(6 * nf);
    this.updateStiffness();
  }

  get nf(): number { return this.csr.nf; }
  /** Iterations since the last begin(). */
  get iterations(): number { return this.work.s[PCG_ITERS]; }

  /** Recomputes bond stiffness from damage/alive, reassembles K and the preconditioner. Call begin() afterwards. */
  updateStiffness(): void {
    computeBondStiffness(this.views, this.damage, this.alive, this.kd, this.bondK, this.m);
    assembleBlockCsr(this.csr.values, this.csr.bondSlot, this.bondK, this.m);
    invertDiagBlocks(this.csr.values, this.csr.diagSlot, this.work.minv, this.csr.nf);
  }

  /** Sets loads from a full vector f (6n,: force N, moment N m per chunk; support rows ignored) and restarts PCG from the current u. */
  begin(f: Float64Array): void {
    const x = this.work.x;
    for (let i = 0; i < this.n; i++) {
      const r = this.freeIdx[i];
      if (r < 0) continue;
      // Detached chunks: zero load and u, so their (unsupported, singular) sub-block stays at the trivial solution.
      if (this.active[i]) for (let k = 0; k < 6; k++) this.fFree[6 * r + k] = f[6 * i + k];
      else for (let k = 0; k < 6; k++) { this.fFree[6 * r + k] = 0; x[6 * r + k] = 0; }
    }
    pcgBegin(this.csr.rowPtr, this.csr.colIdx, this.csr.values, this.fFree, this.work, this.csr.nf);
  }

  /** Runs up to maxIters iterations (one time slice). @returns true when |r| <= tol |f|. */
  iterate(maxIters: number, tol = 1e-8): boolean {
    return pcgIterate(this.csr.rowPtr, this.csr.colIdx, this.csr.values, this.work, this.csr.nf, maxIters, tol) === 1;
  }

  /** Iterates in slices of `slice` until converged or `budgetMs` elapsed (frame budget). @returns converged. */
  iterateBudget(budgetMs: number, tol = 1e-8, slice = 8): boolean {
    const end = performance.now() + budgetMs;
    let done = false;
    while (!done && performance.now() < end) done = this.iterate(slice, tol);
    return done;
  }

  /** Clears the warm start (u = 0). */
  reset(): void { this.work.x.fill(0); }

  /** Writes full displacements u (6n,: translation m, rotation vector rad), supports = 0. */
  displacement(out: Float64Array): void {
    const x = this.work.x;
    for (let i = 0; i < this.n; i++) {
      const r = this.freeIdx[i];
      for (let k = 0; k < 6; k++) out[6 * i + k] = r >= 0 ? x[6 * r + k] : 0;
    }
  }

  /** Bond forces [N, V1, V2, Tq, M1, M2] (6m,) for the current u; `uScratch` (6n,) receives the full u. */
  bondForces(out: Float64Array, uScratch: Float64Array): void {
    this.displacement(uScratch);
    computeBondForces(this.views, this.kd, uScratch, out, this.m);
  }
}
