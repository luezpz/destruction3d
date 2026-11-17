// Fake impact deformation for `deform: 'offsets'` materials (NOTES Design §4 "Fake deformation"). Headless, runs in the integrity worker.
// Each attached chunk keeps a permanent visual offset o_i (3,) and tilt th_i (3,, world rotation vector); colliders stay at rest.
// Impact at chunk h, point p, unit dir n, impulse J (N s), within graph radius R (bond hops) of h:
//   s = oMax h_size (exp(-Jacc_h / jRef) - exp(-(Jacc_h + J) / jRef)), Jacc_h = impulse already taken by h: the increment of a
//   saturating curve, so one contact split over many substep events dents like one hit and repeated hits dent less;  phi(x) = (1 - x^2)^3,  w_i = phi(g_i / R)  (g_i = hop distance)
//   o_i += s w_i n                       (|o_i| clamped to oMax size_i)
//   th_i += k_t w_i s (n x (c_i - p)) / (R h_size)^2   (|th_i| clamped to oMax rad)
// Cracks from the offset gradient: per bond, w_b = |o_a - o_b| / (wMax size_ab), d_b = max(d_b, w_b); d_b >= 1 breaks the bond.
// Alloc-free per impact: BFS queue, visit marks and outputs are preallocated.

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import type { BondAdjacency } from '../kernels/bondAdjacency';

export class DentField {
  /** Visual offset (3n,) and tilt (3n,) per chunk, structure frame. */
  readonly offset: Float64Array;
  readonly tilt: Float64Array;
  /** Crack damage per bond (m,), monotone. */
  readonly damage: Float64Array;
  /** Chunks whose offset/tilt changed since the last `clearDirty`: dirty[0 .. dirtyCount). */
  readonly dirty: Uint32Array;
  dirtyCount = 0;
  /** Bonds that reached d >= 1 since the last `clearBreaks`: breaks[0 .. breakCount). */
  readonly breaks: Uint32Array;
  breakCount = 0;
  /** Impulse taken so far per hit chunk (N s): (n,). */
  readonly impulse: Float64Array;
  private readonly isDirty: Uint8Array;
  private readonly size: Float64Array;
  private readonly hop: Int32Array;
  private readonly queue: Uint32Array;
  private readonly dentable: Uint8Array;
  private readonly bondQueued: Uint8Array;

  constructor(private readonly model: ChunkModel, private readonly table: MaterialTable, private readonly adj: BondAdjacency) {
    const n = model.chunkCount, m = model.bondCount;
    this.offset = new Float64Array(3 * n); this.tilt = new Float64Array(3 * n);
    this.damage = new Float64Array(m);
    this.dirty = new Uint32Array(n); this.isDirty = new Uint8Array(n);
    this.breaks = new Uint32Array(m); this.bondQueued = new Uint8Array(m);
    this.size = new Float64Array(n); this.hop = new Int32Array(n).fill(-1); this.queue = new Uint32Array(n);
    this.dentable = new Uint8Array(n);
    this.impulse = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      this.size[i] = Math.cbrt(model.chunkVolume[i]);
      this.dentable[i] = table.materials[model.chunkMaterial[i]].deform === 'offsets' ? 1 : 0;
    }
  }

  /**
   * Applies one impact. @param hit chunk hit. @param p contact point (3,) at p[po..]. @param n unit push direction (3,) at n[no..].
   * @param J impulse (N s). @param force peak contact force (N), gated by the material's dent.minForce.
   * @param attached (n,) 1 = chunk still in the static structure. @param alive (m,) bond liveness.
   */
  apply(hit: number, p: ArrayLike<number>, po: number, n: ArrayLike<number>, no: number, J: number, force: number,
    attached: Uint8Array, alive: Uint8Array): void {
    const { model, adj, hop, queue, size } = this;
    if (!this.dentable[hit] || !attached[hit]) return;
    const dp = this.table.materials[model.chunkMaterial[hit]].dent;
    if (force < dp.minForce) return;
    const R = dp.radius, h = size[hit];
    const j0 = this.impulse[hit];
    this.impulse[hit] = j0 + J;
    const s = dp.oMax * h * (Math.exp(-j0 / dp.jRef) - Math.exp(-(j0 + J) / dp.jRef));
    const nx = n[no], ny = n[no + 1], nz = n[no + 2], tScale = (dp.tilt * s) / ((R * h) * (R * h));
    let head = 0, tail = 0;
    queue[tail++] = hit; hop[hit] = 0;
    while (head < tail) {
      const i = queue[head++], g = hop[i];
      const x = g / R, w = (1 - x * x) ** 3;
      const lim = this.table.materials[model.chunkMaterial[i]].dent.oMax;
      const i3 = 3 * i, o = this.offset, t = this.tilt;
      o[i3] += s * w * nx; o[i3 + 1] += s * w * ny; o[i3 + 2] += s * w * nz;
      clampLen(o, i3, lim * size[i]);
      const cx = model.chunkPos[i3] - p[po], cy = model.chunkPos[i3 + 1] - p[po + 1], cz = model.chunkPos[i3 + 2] - p[po + 2];
      t[i3] += tScale * w * (ny * cz - nz * cy); t[i3 + 1] += tScale * w * (nz * cx - nx * cz); t[i3 + 2] += tScale * w * (nx * cy - ny * cx);
      clampLen(t, i3, lim);
      if (!this.isDirty[i]) { this.isDirty[i] = 1; this.dirty[this.dirtyCount++] = i; }
      for (let k = adj.adjOffset[i]; k < adj.adjOffset[i + 1]; k++) {
        const b = adj.adjBond[k];
        if (!alive[b]) continue;
        const j = model.bondA[b] === i ? model.bondB[b] : model.bondA[b];
        if (g + 1 < R && hop[j] < 0 && this.dentable[j] && attached[j]) { hop[j] = g + 1; queue[tail++] = j; }
      }
    }
    // crack damage on every alive bond touching a moved chunk (ring bonds included), then reset the BFS marks
    for (let q = 0; q < tail; q++) {
      const i = queue[q];
      for (let k = adj.adjOffset[i]; k < adj.adjOffset[i + 1]; k++) {
        const b = adj.adjBond[k];
        if (!alive[b] || this.bondQueued[b]) continue;
        const a = model.bondA[b], c = model.bondB[b];
        const wMax = this.table.materials[model.bondMaterial[b]].dent.wMax * 0.5 * (size[a] + size[c]);
        const dx = this.offset[3 * a] - this.offset[3 * c], dy = this.offset[3 * a + 1] - this.offset[3 * c + 1], dz = this.offset[3 * a + 2] - this.offset[3 * c + 2];
        const d = Math.hypot(dx, dy, dz) / wMax;
        if (d > this.damage[b]) this.damage[b] = d;
        if (this.damage[b] >= 1) { this.bondQueued[b] = 1; this.breaks[this.breakCount++] = b; }
      }
    }
    for (let q = 0; q < tail; q++) hop[queue[q]] = -1;
  }

  /** Forgets the dirty list (after the poses were copied into a reply). */
  clearDirty(): void {
    for (let k = 0; k < this.dirtyCount; k++) this.isDirty[this.dirty[k]] = 0;
    this.dirtyCount = 0;
  }

  /** Forgets the break list (after the bonds were removed from the graph). */
  clearBreaks(): void { this.breakCount = 0; }
}

/** Scales v[o..o+3) down to length <= max. */
function clampLen(v: Float64Array, o: number, max: number): void {
  const l = Math.hypot(v[o], v[o + 1], v[o + 2]);
  if (l > max) { const f = max / l; v[o] *= f; v[o + 1] *= f; v[o + 2] *= f; }
}
