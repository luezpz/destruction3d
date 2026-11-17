// Static structure bond graph (NOTES Design §2): alive flags, chunk -> bond adjacency, and tier 0 connectivity on removals.
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
  /** Components (after this call's breaks) of floating parts that lost a bond: floatChunks[floatOffset[k] .. [k+1]). A part split in two shows as two. */
  readonly floatChunks: Uint32Array;
  readonly floatOffset: Uint32Array;
  floatCount = 0;
  private readonly staticBroken: Uint32Array;
  private readonly floatBroken: Uint32Array;

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
    this.floatChunks = new Uint32Array(n);
    this.floatOffset = new Uint32Array(n + 1);
    this.staticBroken = new Uint32Array(m);
    this.floatBroken = new Uint32Array(m);
  }

  /**
   * Removes bonds (scripted cuts, joint breaks, or breaks reported by a structure solver), then runs tier 0 on the static
   * structure and collects the components of floating parts that lost bonds (island re-fracture). Skips already-broken bonds.
   * @returns detached components.
   */
  removeBonds(bonds: ArrayLike<number>): number {
    let nb = 0, ns = 0, nf = 0;
    for (let k = 0; k < bonds.length; k++) {
      const b = bonds[k];
      if (!this.alive[b]) continue;
      this.alive[b] = 0;
      this.broken[nb++] = b;
      // both ends share the state: an alive bond never links an attached chunk to a detached one
      if (this.attached[this.model.bondA[b]]) this.staticBroken[ns++] = b; else this.floatBroken[nf++] = b;
    }
    this.brokenCount = nb;
    this.floatCount = this.connectivity.collectComponents(this.floatBroken, nf, this.alive, this.floatChunks, this.floatOffset);
    this.detachedCount = this.connectivity.findDetached(this.staticBroken, ns, this.alive, this.detachedChunks, this.detachedOffset);
    const end = this.detachedOffset[this.detachedCount];
    for (let k = 0; k < end; k++) this.attached[this.detachedChunks[k]] = 0;
    return this.detachedCount;
  }
}
