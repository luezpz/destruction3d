// Impact loads from physics contacts (PLAN §2): contact force on a structure chunk -> static load for tier 1.
// Per chunk we keep the peak force since the last tier-1 tick (physics may step several times per tick), scale it by
// the dynamic amplification (x2) and add it to f. Short spikes are rejected later by the ratio low-pass, not here.
// Headless: receives plain numbers (chunk id, force vector), not physics events.

/** Dynamic amplification of a suddenly applied load (PLAN §2). */
export const IMPACT_AMPLIFICATION = 2;

export class ImpactLoads {
  /** Peak force vector per chunk since the last `applyAndClear` (3n,), N. */
  readonly peak: Float64Array;
  private readonly peakMag2: Float64Array;
  /** Time (s) since each chunk's peak was set (n,); used by holders that keep peaks for a window. */
  private readonly age: Float64Array;
  private readonly touched: Uint32Array;
  private touchedCount = 0;

  constructor(n: number) {
    this.peak = new Float64Array(3 * n);
    this.peakMag2 = new Float64Array(n);
    this.touched = new Uint32Array(n);
    this.age = new Float64Array(n);
  }

  /** Records force (fx, fy, fz) N acting on chunk c; keeps the largest by magnitude since the last apply. */
  add(c: number, fx: number, fy: number, fz: number): void {
    const m2 = fx * fx + fy * fy + fz * fz;
    if (m2 <= this.peakMag2[c]) return;
    if (this.peakMag2[c] === 0) this.touched[this.touchedCount++] = c;
    this.peakMag2[c] = m2;
    this.age[c] = 0;
    this.peak[3 * c] = fx; this.peak[3 * c + 1] = fy; this.peak[3 * c + 2] = fz;
  }

  /**
   * Records a contact between chunks a, b (-1 = not a structure chunk) with force magnitude F along dir (A -> B).
   * Force on B = +dir F, on A = -dir F. @param isStatic (n,) 1 for chunks still in the static solve.
   */
  addContact(a: number, b: number, F: number, dx: number, dy: number, dz: number, isStatic: Uint8Array): void {
    if (b >= 0 && isStatic[b]) this.add(b, dx * F, dy * F, dz * F);
    if (a >= 0 && isStatic[a]) this.add(a, -dx * F, -dy * F, -dz * F);
  }

  /** Merges these peaks into `dst` (max by magnitude per chunk) and clears them. @returns chunks moved. */
  moveInto(dst: ImpactLoads): number {
    const n = this.touchedCount;
    for (let k = 0; k < n; k++) {
      const c = this.touched[k];
      dst.add(c, this.peak[3 * c], this.peak[3 * c + 1], this.peak[3 * c + 2]);
      this.peak[3 * c] = this.peak[3 * c + 1] = this.peak[3 * c + 2] = 0;
      this.peakMag2[c] = 0;
    }
    this.touchedCount = 0;
    return n;
  }

  /** Adds amplified peak forces into f (6n,: force rows of each chunk) without clearing. @returns chunks loaded. */
  apply(f: Float64Array): number {
    for (let k = 0; k < this.touchedCount; k++) {
      const c = this.touched[k];
      for (let a = 0; a < 3; a++) f[6 * c + a] += IMPACT_AMPLIFICATION * this.peak[3 * c + a];
    }
    return this.touchedCount;
  }

  /** Ages all held peaks by dt (s). */
  advance(dt: number): void {
    for (let k = 0; k < this.touchedCount; k++) this.age[this.touched[k]] += dt;
  }

  /** Drops peaks older than `hold` s (compacts the touched list). @returns peaks still held. */
  dropOlderThan(hold: number): number {
    let w = 0;
    for (let k = 0; k < this.touchedCount; k++) {
      const c = this.touched[k];
      if (this.age[c] < hold) { this.touched[w++] = c; continue; }
      this.peak[3 * c] = this.peak[3 * c + 1] = this.peak[3 * c + 2] = 0;
      this.peakMag2[c] = 0;
    }
    this.touchedCount = w;
    return w;
  }

  /** Adds amplified peak forces into f (6n,: force rows of each chunk) and clears the peaks. @returns chunks loaded. */
  applyAndClear(f: Float64Array): number {
    const n = this.touchedCount;
    for (let k = 0; k < n; k++) {
      const c = this.touched[k];
      for (let a = 0; a < 3; a++) { f[6 * c + a] += IMPACT_AMPLIFICATION * this.peak[3 * c + a]; this.peak[3 * c + a] = 0; }
      this.peakMag2[c] = 0;
    }
    this.touchedCount = 0;
    return n;
  }
}
