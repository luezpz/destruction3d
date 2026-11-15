// Static structure bond graph (PLAN §2): alive flags, chunk -> bond adjacency, and tier 0 connectivity on removals.
// This is the input/output seam for an external structure solver: it reads the graph (model + alive + materials,
// masses) and reports bonds to break; the graph turns breaks into detached components. Setup allocates; queries do not.

import { ChunkFlag, ChunkModel } from '../model/ChunkModel';
import { BondAdjacency, buildBondAdjacency } from '../kernels/bondAdjacency';
import { Connectivity } from './connectivity';

export class StructureGraph {
  readonly adj: BondAdjacency;
  readonly connectivity: Connectivity;
  /** Per bond (m,): 1 = intact. */
  readonly alive: Uint8Array;
  /** Per chunk (n,): 1 while still connected to a support (part of the static structure). */
  readonly attached: Uint8Array;
  /** Bonds removed by the last `removeBonds`: broken[0 .. brokenCount). */
  readonly broken: Uint32Array;
  brokenCount = 0;
  /** Detached chunks of the last call, grouped by component k: detachedChunks[detachedOffset[k] .. detachedOffset[k+1]). */
  readonly detachedChunks: Uint32Array;
  readonly detachedOffset: Uint32Array;
  detachedCount = 0;

  constructor(readonly model: ChunkModel) {
    const n = model.chunkCount, m = model.bondCount;
    this.adj = buildBondAdjacency(n, model.bondA, model.bondB, m);
    const support = Uint8Array.from(model.chunkFlags, (f) => (f & ChunkFlag.SUPPORT ? 1 : 0));
    this.connectivity = new Connectivity(n, model.bondA, model.bondB, this.adj, support);
    this.alive = new Uint8Array(m).fill(1);
    this.attached = new Uint8Array(n).fill(1);
    this.broken = new Uint32Array(m);
    this.detachedChunks = new Uint32Array(n);
    this.detachedOffset = new Uint32Array(n + 1);
  }

  /**
   * Removes bonds (scripted cuts, or breaks reported by a structure solver), then runs tier 0. Skips already-broken
   * bonds and bonds of detached chunks (island re-fracture is not supported yet). @returns detached components.
   */
  removeBonds(bonds: ArrayLike<number>): number {
    let nb = 0;
    for (let k = 0; k < bonds.length; k++) {
      const b = bonds[k];
      if (!this.alive[b] || !this.attached[this.model.bondA[b]]) continue;
      this.alive[b] = 0;
      this.broken[nb++] = b;
    }
    this.brokenCount = nb;
    this.detachedCount = this.connectivity.findDetached(this.broken, nb, this.alive, this.detachedChunks, this.detachedOffset);
    const end = this.detachedOffset[this.detachedCount];
    for (let k = 0; k < end; k++) this.attached[this.detachedChunks[k]] = 0;
    return this.detachedCount;
  }
}
