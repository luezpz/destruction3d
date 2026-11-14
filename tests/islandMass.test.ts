import { describe, expect, it } from 'vitest';
import { islandMassProperties } from '../src/integrity/islandMass';

/** Rotates v by unit quaternion q = (x,y,z,w). */
function rot(q: number[], v: number[]): number[] {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

describe('island mass properties', () => {
  it('two point masses on a diagonal: COM, principal values, axis', () => {
    const pos = Float64Array.of(0, 0, 0, 1, 1, 0), mass = Float64Array.of(2, 2), j = Float64Array.of(0.1, 0.1);
    const mp = islandMassProperties([0, 1], 0, 2, mass, j, pos);
    expect(mp.mass).toBe(4);
    expect(mp.com).toEqual([0.5, 0.5, 0]);
    // d = +-(0.5,0.5,0): along axis (1,1,0)/sqrt2 inertia = 0.2; perpendicular in-plane and z: 0.2 + 4*0.5 = 2.2
    const sorted = [...mp.inertia].sort((a, b) => a - b);
    expect(sorted[0]).toBeCloseTo(0.2, 12);
    expect(sorted[1]).toBeCloseTo(2.2, 12);
    expect(sorted[2]).toBeCloseTo(2.2, 12);
    const k = mp.inertia.findIndex((v) => Math.abs(v - 0.2) < 1e-9);
    const e = [0, 0, 0]; e[k] = 1;
    const axis = rot(mp.frame, e);
    expect(Math.abs(axis[0] * Math.SQRT1_2 + axis[1] * Math.SQRT1_2)).toBeCloseTo(1, 10);
    expect(Math.hypot(...mp.frame)).toBeCloseTo(1, 12);
  });

  it('reconstructs a general tensor: R diag(I) R^T matches the direct sum', () => {
    const n = 7, pos = new Float64Array(3 * n), mass = new Float64Array(n), j = new Float64Array(n);
    let seed = 3;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
    for (let i = 0; i < n; i++) { mass[i] = 1 + rnd(); j[i] = 0.05 * rnd(); for (let a = 0; a < 3; a++) pos[3 * i + a] = rnd() * 3; }
    const ids = Array.from({ length: n }, (_, i) => i);
    const mp = islandMassProperties(ids, 0, n, mass, j, pos);
    for (let a = 0; a < 3; a++) {
      for (let b = 0; b < 3; b++) {
        let direct = 0;
        for (let i = 0; i < n; i++) {
          const d = [0, 1, 2].map((c) => pos[3 * i + c] - mp.com[c]);
          direct += (a === b ? j[i] + mass[i] * (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) : 0) - mass[i] * d[a] * d[b];
        }
        // (R diag R^T)[a][b] = sum_k I_k R[a][k] R[b][k], R[:,k] = rot(q, e_k)
        let rec = 0;
        for (let k = 0; k < 3; k++) {
          const e = [0, 0, 0]; e[k] = 1;
          const c = rot(mp.frame, e);
          rec += mp.inertia[k] * c[a] * c[b];
        }
        expect(rec).toBeCloseTo(direct, 9);
      }
    }
  });
});
