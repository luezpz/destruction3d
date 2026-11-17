// Fake impact deformation for `deform: 'offsets'` materials (NOTES Design §4 "Fake deformation"). Headless, runs in the integrity worker.
// Only the hit ("damaged") chunk moves: no spreading to neighbours and no crack damage, so dents never cause fracture events.
// Each attached chunk keeps damage points D_i, a permanent visual offset o_i (3,) and tilt th_i (3,, world rotation vector);
// colliders stay at rest. Impact on chunk i with unit push dir n (3,) and impulse J (N s):
//   D_i += J / jRef,   f(D) = 1 - exp(-D) (saturates: more damage -> more displaced/rotated, bounded),   df = f(D_new) - f(D_old)
//   o_i  += oMax size_i df n             (along the incoming damage direction; |o_i| clamped to oMax size_i)
//   th_i += tilt df a,  a = random unit axis per hit   (|th_i| clamped to tilt rad)
// Alloc-free per impact; the axis comes from a seeded PRNG so runs are reproducible.

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';

export class DentField {
  /** Visual offset (3n,) and tilt (3n,) per chunk, structure frame. */
  readonly offset: Float64Array;
  readonly tilt: Float64Array;
  /** Damage points per chunk (n,): accumulated impulse / jRef, monotone. */
  readonly points: Float64Array;
  /** Chunks whose offset/tilt changed since the last `clearDirty`: dirty[0 .. dirtyCount). */
  readonly dirty: Uint32Array;
  dirtyCount = 0;
  private readonly isDirty: Uint8Array;
  private readonly size: Float64Array;
  private readonly dentable: Uint8Array;
  /** xorshift32 state for the random tilt axes. */
  private seed: number;

  /** @param model chunk geometry/materials. @param table materials (dent records). @param seed PRNG seed (nonzero). */
  constructor(private readonly model: ChunkModel, private readonly table: MaterialTable, seed = 0x9e3779b9) {
    const n = model.chunkCount;
    this.offset = new Float64Array(3 * n); this.tilt = new Float64Array(3 * n);
    this.points = new Float64Array(n);
    this.dirty = new Uint32Array(n); this.isDirty = new Uint8Array(n);
    this.size = new Float64Array(n); this.dentable = new Uint8Array(n);
    this.seed = seed >>> 0 || 1;
    for (let i = 0; i < n; i++) {
      this.size[i] = Math.cbrt(model.chunkVolume[i]);
      this.dentable[i] = table.materials[model.chunkMaterial[i]].deform === 'offsets' ? 1 : 0;
    }
  }

  /**
   * Applies one impact to the hit chunk only. @param hit chunk hit. @param n unit push direction (3,) at n[no..].
   * @param J impulse (N s). @param force peak contact force (N), gated by the material's dent.minForce.
   * @param attached (n,) 1 = chunk still in the static structure.
   */
  apply(hit: number, n: ArrayLike<number>, no: number, J: number, force: number, attached: Uint8Array): void {
    if (!this.dentable[hit] || !attached[hit] || !(J > 0)) return;
    const dp = this.table.materials[this.model.chunkMaterial[hit]].dent;
    if (force < dp.minForce) return;
    const d0 = this.points[hit], d1 = d0 + J / dp.jRef;
    this.points[hit] = d1;
    const df = Math.exp(-d0) - Math.exp(-d1);
    const i3 = 3 * hit, o = this.offset, t = this.tilt, s = dp.oMax * this.size[hit] * df;
    o[i3] += s * n[no]; o[i3 + 1] += s * n[no + 1]; o[i3 + 2] += s * n[no + 2];
    clampLen(o, i3, dp.oMax * this.size[hit]);
    // uniform random axis on the sphere: z in [-1, 1], phi in [0, 2pi)
    const z = 2 * this.rand() - 1, phi = 2 * Math.PI * this.rand(), r = Math.sqrt(1 - z * z), a = dp.tilt * df;
    t[i3] += a * r * Math.cos(phi); t[i3 + 1] += a * r * Math.sin(phi); t[i3 + 2] += a * z;
    clampLen(t, i3, dp.tilt);
    if (!this.isDirty[hit]) { this.isDirty[hit] = 1; this.dirty[this.dirtyCount++] = hit; }
  }

  /** Forgets the dirty list (after the poses were copied into a reply). */
  clearDirty(): void {
    for (let k = 0; k < this.dirtyCount; k++) this.isDirty[this.dirty[k]] = 0;
    this.dirtyCount = 0;
  }

  /** @returns uniform [0, 1) from xorshift32 (deterministic, no allocation). */
  private rand(): number {
    let x = this.seed;
    x ^= x << 13; x >>>= 0; x ^= x >>> 17; x ^= x << 5; x >>>= 0;
    this.seed = x;
    return x / 4294967296;
  }
}

/** Scales v[o..o+3) down to length <= max. */
function clampLen(v: Float64Array, o: number, max: number): void {
  const l = Math.hypot(v[o], v[o + 1], v[o + 2]);
  if (l > max) { const f = max / l; v[o] *= f; v[o + 1] *= f; v[o + 2] *= f; }
}
