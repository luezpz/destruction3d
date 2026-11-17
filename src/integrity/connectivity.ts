// Tier 0 connectivity (NOTES Design §2): after bond breaks, find chunks that lost every path to a support.
// Bidirectional-in-spirit: BFS from each broken bond's ends, stopping as soon as a support (or an already-proven
// grounded chunk) is reached. Cost is O(size of the detached part) when a piece falls, small when it does not.
// Setup allocates; queries do not.

import { BondAdjacency } from '../kernels/bondAdjacency';

export class Connectivity {
  /** Per chunk: 1 = support. */
  private readonly support: Uint8Array;
  /** Visit stamp per chunk (avoids clearing), BFS queue, per-chunk result stamp of grounded proofs. */
  private readonly mark: Uint32Array;
  private readonly grounded: Uint32Array;
  private readonly queue: Uint32Array;
  private stamp = 0;
  /** Component id of every detached chunk from the last `findDetached` (valid where `detachedOf` lists it). */
  readonly componentOf: Int32Array;

  /** @param support (n,) 1 where the chunk is anchored. */
  constructor(n: number, private readonly bondA: Uint32Array, private readonly bondB: Uint32Array,
    private readonly adj: BondAdjacency, support: Uint8Array) {
    this.support = support;
    this.mark = new Uint32Array(n);
    this.grounded = new Uint32Array(n);
    this.queue = new Uint32Array(n);
    this.componentOf = new Int32Array(n).fill(-1);
  }

  /**
   * Given broken bonds (and the current `alive` flags), writes newly unsupported chunks to `outChunks` grouped by
   * component: component k is outChunks[outOffset[k] .. outOffset[k+1]). Chunks already in a floating component
   * (no support at all before the break) are reported too, so callers should only pass bonds of the static structure.
   * @returns number of components found.
   */
  findDetached(broken: Uint32Array, nBroken: number, alive: Uint8Array, outChunks: Uint32Array, outOffset: Uint32Array): number {
    const s = ++this.stamp;
    let comps = 0, written = 0;
    outOffset[0] = 0;
    for (let q = 0; q < nBroken; q++) {
      for (let side = 0; side < 2; side++) {
        const start = side ? this.bondB[broken[q]] : this.bondA[broken[q]];
        if (this.mark[start] === s) continue; // already in a found component or proven grounded
        const base = written;
        const hitSupport = this.bfs(start, s, alive, outChunks, written);
        const size = this.lastSize;
        if (hitSupport) {
          for (let k = 0; k < size; k++) this.grounded[outChunks[base + k]] = s;
        } else {
          for (let k = 0; k < size; k++) this.componentOf[outChunks[base + k]] = comps;
          written += size;
          outOffset[++comps] = written;
        }
      }
    }
    return comps;
  }

  /**
   * Full components (no support early-exit) of both ends of each bond, deduplicated: component k is
   * outChunks[outOffset[k] .. outOffset[k+1]). For breaks inside floating (already detached) parts, where any split must be found.
   * @returns number of components.
   */
  collectComponents(bonds: Uint32Array, nBonds: number, alive: Uint8Array, outChunks: Uint32Array, outOffset: Uint32Array): number {
    const s = ++this.stamp;
    let comps = 0, written = 0;
    outOffset[0] = 0;
    for (let q = 0; q < nBonds; q++) {
      for (let side = 0; side < 2; side++) {
        const start = side ? this.bondB[bonds[q]] : this.bondA[bonds[q]];
        if (this.mark[start] === s) continue;
        this.bfs(start, s, alive, outChunks, written); // floating parts hold no support and no grounded stamp s: full BFS
        written += this.lastSize;
        outOffset[++comps] = written;
      }
    }
    return comps;
  }

  private lastSize = 0;

  /** BFS over alive bonds from `start`, writing visited chunks into out[at..]; stops early at a support/grounded chunk. */
  private bfs(start: number, s: number, alive: Uint8Array, out: Uint32Array, at: number): boolean {
    let head = 0, tail = 0;
    this.queue[tail++] = start;
    this.mark[start] = s;
    let found = false;
    while (head < tail && !found) {
      const c = this.queue[head++];
      if (this.support[c] || this.grounded[c] === s) { found = true; break; }
      for (let k = this.adj.adjOffset[c]; k < this.adj.adjOffset[c + 1]; k++) {
        const b = this.adj.adjBond[k];
        if (!alive[b]) continue;
        const o = this.bondA[b] === c ? this.bondB[b] : this.bondA[b];
        if (this.mark[o] === s) {
          if (this.grounded[o] === s) { found = true; break; }
          continue;
        }
        this.mark[o] = s;
        this.queue[tail++] = o;
      }
    }
    for (let k = 0; k < tail; k++) out[at + k] = this.queue[k];
    this.lastSize = tail;
    return found;
  }
}
