// Integrity work behind a message boundary (PLAN §7): the same core runs inline (tests, fallback) or in a worker.
// Main -> core: loads f (transferred, returned in the reply = double buffering), dt, scripted bond removals.
// Core -> main: bonds broken, detached components, filtered ratios (Float32 for colouring), graph version.
// The graph version increments on every break; replies always carry breaks (facts), so stale loads never lose them.

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import { BondSolver } from './BondSolver';
import { StressTier } from './StressTier';

export interface TickRequest {
  type: 'tick';
  /** Seconds since the previous request (damage time). */
  dt: number;
  /** Loads (6n,): force N, moment N m per chunk. Ownership passes to the core and comes back in the reply. */
  f: Float64Array;
  /** Bonds to remove before solving (scripted cuts). */
  remove: Uint32Array;
  /** Solver time budget for this request (ms) and PCG tolerance. */
  budgetMs: number;
  tol: number;
}

export interface TickReply {
  type: 'result';
  version: number;
  /** The request's load buffer, returned for reuse. */
  f: Float64Array;
  converged: boolean;
  /** Bonds removed by this request (removals first, then tier-1 breaks). */
  broken: Uint32Array;
  /** Newly detached components: chunks of component k are detachedChunks[detachedOffset[k] .. detachedOffset[k+1]). */
  detachedChunks: Uint32Array;
  detachedOffset: Uint32Array;
  /** Filtered failure ratio per bond (m,), for display. */
  ratio: Float32Array;
}

export interface InitRequest {
  type: 'init';
  model: ChunkModel;
  table: MaterialTable;
}

/** Transferable buffers of a reply (for postMessage). */
export function replyTransfer(r: TickReply): ArrayBuffer[] {
  return [r.f.buffer, r.broken.buffer, r.detachedChunks.buffer, r.detachedOffset.buffer, r.ratio.buffer] as ArrayBuffer[];
}

export class IntegrityCore {
  readonly solver: BondSolver;
  readonly tier: StressTier;
  version = 0;
  private sinceTier = 0;
  /** A PCG solve is in progress on `solveF` (restarting every request stalls CG on cracked, ill-conditioned lattices). */
  private solving = false;
  private readonly solveF: Float64Array;
  // Per-request accumulation (setup-time sized; reply copies are small and transferred).
  private readonly brokenAcc: number[] = [];
  private readonly compChunks: number[] = [];
  private readonly compOffset: number[] = [0];

  constructor(readonly model: ChunkModel, table: MaterialTable) {
    this.solver = new BondSolver(model, table);
    this.tier = new StressTier(model, table, this.solver);
    this.solveF = new Float64Array(6 * model.chunkCount);
  }

  /** Copies the tier's last broken list and detached components into the per-request accumulators. */
  private collect(): void {
    const t = this.tier;
    for (let k = 0; k < t.brokenCount; k++) this.brokenAcc.push(t.broken[k]);
    for (let c = 0; c < t.detachedCount; c++) {
      for (let q = t.detachedOffset[c]; q < t.detachedOffset[c + 1]; q++) this.compChunks.push(t.detachedChunks[q]);
      this.compOffset.push(this.compChunks.length);
    }
    if (t.brokenCount > 0) this.version++;
  }

  /** Handles one tick: removals, (re)start solve if idle, budgeted iterations, tier 1/0 on convergence. */
  tick(req: TickRequest): TickReply {
    this.brokenAcc.length = 0; this.compChunks.length = 0; this.compOffset.length = 1;
    this.sinceTier += req.dt;
    if (req.remove.length) {
      this.tier.removeBonds(req.remove);
      this.collect();
    }
    // Continue the running solve while loads and stiffness are unchanged; otherwise restart PCG on the latest loads
    // (warm-started from the current u), so a long solve never hides newer impacts.
    if (!this.solving || req.remove.length || !sameLoads(req.f, this.solveF)) {
      this.solveF.set(req.f);
      this.solver.begin(req.f);
      this.solving = true;
    }
    const converged = this.solver.iterateBudget(req.budgetMs, req.tol);
    if (converged) {
      this.solving = false;
      this.tier.step(this.sinceTier);
      this.sinceTier = 0;
      this.collect();
    }
    return {
      type: 'result', version: this.version, f: req.f, converged,
      broken: Uint32Array.from(this.brokenAcc), detachedChunks: Uint32Array.from(this.compChunks),
      detachedOffset: Uint32Array.from(this.compOffset), ratio: Float32Array.from(this.tier.ratioFiltered),
    };
  }
}

/** True when load vectors a, b (6n,) are equal (exact: loads are rebuilt from the same gravity + held impacts). */
function sameLoads(a: Float64Array, b: Float64Array): boolean {
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}
