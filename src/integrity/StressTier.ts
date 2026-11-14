// Tier 1 stress step (PLAN §2): solver forces -> failure ratios -> damage -> removed bonds -> tier 0 connectivity.
// Owns the per-bond tier-1 buffers; setup allocates, `step` does not.

import { ChunkFlag, ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { MaterialViews, computeFailureRatios } from '../kernels/failureRatio';
import { BondAdjacency, buildBondAdjacency, updateDamage } from '../kernels/damage';
import { BondSolver } from './BondSolver';
import { Connectivity } from './connectivity';

/** Packs the material fields used by the tier-1 kernels by material id (sigmaC 0 resolved to sigmaT). */
export function materialViews(table: MaterialTable): MaterialViews {
  const ms = table.materials, f = (fn: (i: number) => number) => Float64Array.from(ms, (_, i) => fn(i));
  const grainAxis = new Float64Array(3 * ms.length);
  ms.forEach((m, i) => { if (m.grain) grainAxis.set(m.grain.axis, 3 * i); });
  return {
    E: f((i) => ms[i].E), sigmaT: f((i) => ms[i].sigmaT), sigmaC: f((i) => ms[i].sigmaC || ms[i].sigmaT),
    tauC: f((i) => ms[i].tauC), mu: f((i) => ms[i].mu), sigmaY: f((i) => ms[i].sigmaY), H: f((i) => ms[i].H),
    epsElasticMax: f((i) => ms[i].epsElasticMax), damageRate: f((i) => ms[i].damageRate),
    filterTime: f((i) => ms[i].filterTime), propagation: f((i) => ms[i].propagation),
    grainAxis, grainRatio: f((i) => ms[i].grain?.strengthRatio ?? 1),
  };
}

export class StressTier {
  readonly mat: MaterialViews;
  readonly adj: BondAdjacency;
  readonly connectivity: Connectivity;
  /** Bond forces (6m,), raw ratio (m,), filtered ratio (m,), plastic strain (m,, 0 until M4). */
  readonly forces: Float64Array;
  readonly ratio: Float64Array;
  readonly ratioFiltered: Float64Array;
  readonly plasticStrain: Float64Array;
  /** Bonds removed in the last step: broken[0 .. brokenCount). */
  readonly broken: Uint32Array;
  brokenCount = 0;
  /** Detached chunks of the last step, grouped by component (see Connectivity.findDetached). */
  readonly detachedChunks: Uint32Array;
  readonly detachedOffset: Uint32Array;
  detachedCount = 0;
  private readonly uScratch: Float64Array;

  constructor(private readonly model: ChunkModel, table: MaterialTable, readonly solver: BondSolver) {
    const n = model.chunkCount, m = model.bondCount;
    this.mat = materialViews(table);
    this.adj = buildBondAdjacency(n, model.bondA, model.bondB, m);
    const support = Uint8Array.from(model.chunkFlags, (f) => (f & ChunkFlag.SUPPORT ? 1 : 0));
    this.connectivity = new Connectivity(n, model.bondA, model.bondB, this.adj, support);
    this.forces = new Float64Array(6 * m);
    this.ratio = new Float64Array(m);
    this.ratioFiltered = new Float64Array(m);
    this.plasticStrain = new Float64Array(m);
    this.broken = new Uint32Array(m);
    this.detachedChunks = new Uint32Array(n);
    this.detachedOffset = new Uint32Array(n + 1);
    this.uScratch = new Float64Array(6 * n);
  }

  /**
   * One tier-1 update from the solver's current u, advancing damage by dt (s). On breaks: reassembles K and runs
   * tier 0. Caller then restarts the solve (`solver.begin(f)`). @returns number of bonds broken this step.
   */
  step(dt: number): number {
    const s = this.solver, m = this.model.bondCount;
    s.bondForces(this.forces, this.uScratch);
    computeFailureRatios(this.model, this.mat, this.forces, s.alive, this.plasticStrain, this.ratio, m);
    this.brokenCount = updateDamage(this.mat, this.model.bondMaterial, this.model.bondA, this.model.bondB, this.adj,
      this.ratio, this.ratioFiltered, s.damage, s.alive, dt, this.broken, m);
    this.detachedCount = 0;
    if (this.brokenCount > 0 || this.anyDamageChanged) {
      s.updateStiffness();
    }
    if (this.brokenCount > 0) this.runTier0();
    return this.brokenCount;
  }

  /** Tier 0 on the current broken list; detached chunks leave the static solve (solver.active = 0). */
  private runTier0(): void {
    this.detachedCount = this.connectivity.findDetached(this.broken, this.brokenCount, this.solver.alive, this.detachedChunks, this.detachedOffset);
    const end = this.detachedOffset[this.detachedCount];
    for (let k = 0; k < end; k++) this.solver.active[this.detachedChunks[k]] = 0;
  }

  /** Damage growth changes stiffness even without breaks; v1 always reassembles when any ratio is filtered above 1. */
  private get anyDamageChanged(): boolean {
    for (let b = 0; b < this.model.bondCount; b++) if (this.solver.alive[b] && this.ratioFiltered[b] > 1) return true;
    return false;
  }

  /** Removes bonds directly (e.g. scripted wall removal / explosion), then runs tier 0. @returns detached component count. */
  removeBonds(bonds: ArrayLike<number>): number {
    let nb = 0;
    for (let k = 0; k < bonds.length; k++) {
      const b = bonds[k];
      if (!this.solver.alive[b]) continue;
      this.solver.alive[b] = 0;
      this.broken[nb++] = b;
    }
    this.brokenCount = nb;
    this.solver.updateStiffness();
    this.runTier0();
    return this.detachedCount;
  }
}
