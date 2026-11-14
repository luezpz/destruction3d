// Block-Jacobi preconditioned conjugate gradient on block CSR K (PLAN §2). Time-sliceable: pcgBegin once per
// load/stiffness change, then pcgIterate in bounded slices. Warm start = keep x from the previous solve.
// All vectors (6 nf,); no allocation per call.

import { blockMatVec } from './blockCsr';

/** Indices into the PCG scalar state (Float64Array(PCG_SCALARS)). */
export const PCG_RZ = 0, PCG_BNORM2 = 1, PCG_ITERS = 2, PCG_RNORM2 = 3;
export const PCG_SCALARS = 4;

/** PCG work vectors and scalar state; all (6 nf,) except `minv` (36 nf,) and `s` (PCG_SCALARS,). */
export interface PcgWork {
  x: Float64Array; r: Float64Array; z: Float64Array; p: Float64Array; q: Float64Array;
  minv: Float64Array; s: Float64Array;
}

/** Allocates PCG work buffers for nf free block rows (setup time only). */
export function createPcgWork(nf: number): PcgWork {
  const v = () => new Float64Array(6 * nf);
  return { x: v(), r: v(), z: v(), p: v(), q: v(), minv: new Float64Array(36 * nf), s: new Float64Array(PCG_SCALARS) };
}

const aug = new Float64Array(72); // Gauss-Jordan scratch [A | I] (6,12)

/**
 * minv_i = inverse of diagonal block K_ii (6,6) for each row, by Gauss-Jordan with partial pivoting.
 * A singular block (chunk with no live bonds) gets a zero inverse, so its dofs stay at the initial x.
 */
export function invertDiagBlocks(values: Float64Array, diagSlot: Uint32Array, minv: Float64Array, nf: number): void {
  for (let i = 0; i < nf; i++) {
    const src = 36 * diagSlot[i], dst = 36 * i;
    let scale = 0;
    for (let r = 0; r < 6; r++) {
      for (let c = 0; c < 6; c++) { aug[12 * r + c] = values[src + 6 * r + c]; aug[12 * r + 6 + c] = r === c ? 1 : 0; }
      scale = Math.max(scale, Math.abs(values[src + 7 * r]));
    }
    let ok = scale > 0;
    for (let c = 0; c < 6 && ok; c++) {
      let piv = c;
      for (let r = c + 1; r < 6; r++) if (Math.abs(aug[12 * r + c]) > Math.abs(aug[12 * piv + c])) piv = r;
      const pv = aug[12 * piv + c];
      if (Math.abs(pv) <= 1e-14 * scale) { ok = false; break; }
      if (piv !== c) for (let k = 0; k < 12; k++) { const t = aug[12 * c + k]; aug[12 * c + k] = aug[12 * piv + k]; aug[12 * piv + k] = t; }
      for (let k = 0; k < 12; k++) aug[12 * c + k] /= pv;
      for (let r = 0; r < 6; r++) {
        if (r === c) continue;
        const f = aug[12 * r + c];
        if (f !== 0) for (let k = 0; k < 12; k++) aug[12 * r + k] -= f * aug[12 * c + k];
      }
    }
    for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) minv[dst + 6 * r + c] = ok ? aug[12 * r + 6 + c] : 0;
  }
}

/** z = blockdiag(minv) r. */
function applyPrecond(minv: Float64Array, r: Float64Array, z: Float64Array, nf: number): void {
  for (let i = 0; i < nf; i++) {
    const o = 6 * i, m = 36 * i;
    for (let a = 0; a < 6; a++) {
      const mm = m + 6 * a;
      z[o + a] = minv[mm] * r[o] + minv[mm + 1] * r[o + 1] + minv[mm + 2] * r[o + 2] + minv[mm + 3] * r[o + 3] + minv[mm + 4] * r[o + 4] + minv[mm + 5] * r[o + 5];
    }
  }
}

/** Dot product of the first len entries. */
function dot(a: Float64Array, b: Float64Array, len: number): number {
  let s = 0;
  for (let k = 0; k < len; k++) s += a[k] * b[k];
  return s;
}

/** Starts a solve of K x = f from the current x (warm start): r = f - K x, z = M^-1 r, p = z, resets iteration count. */
export function pcgBegin(rowPtr: Uint32Array, colIdx: Uint32Array, values: Float64Array, f: Float64Array, w: PcgWork, nf: number): void {
  const len = 6 * nf;
  blockMatVec(rowPtr, colIdx, values, w.x, w.q, nf);
  for (let k = 0; k < len; k++) w.r[k] = f[k] - w.q[k];
  applyPrecond(w.minv, w.r, w.z, nf);
  w.p.set(w.z);
  w.s[PCG_RZ] = dot(w.r, w.z, len);
  w.s[PCG_BNORM2] = dot(f, f, len);
  w.s[PCG_RNORM2] = dot(w.r, w.r, len);
  w.s[PCG_ITERS] = 0;
}

/**
 * Runs up to maxIters PCG iterations. Converged when |r| <= tol |f| (tol relative; |f| = 0 -> |r| = 0).
 * @returns 1 if converged, else 0 (call again next slice).
 */
export function pcgIterate(rowPtr: Uint32Array, colIdx: Uint32Array, values: Float64Array, w: PcgWork, nf: number, maxIters: number, tol: number): number {
  const len = 6 * nf, s = w.s, x = w.x, r = w.r, z = w.z, p = w.p, q = w.q;
  const thr = tol * tol * s[PCG_BNORM2];
  for (let it = 0; it < maxIters; it++) {
    if (s[PCG_RNORM2] <= thr) return 1;
    blockMatVec(rowPtr, colIdx, values, p, q, nf);
    const pq = dot(p, q, len);
    if (!(pq > 0)) return 1; // breakdown: p in the null space (no further progress possible)
    const alpha = s[PCG_RZ] / pq;
    let rr = 0;
    for (let k = 0; k < len; k++) { x[k] += alpha * p[k]; r[k] -= alpha * q[k]; rr += r[k] * r[k]; }
    applyPrecond(w.minv, r, z, nf);
    const rz = dot(r, z, len), beta = rz / s[PCG_RZ];
    for (let k = 0; k < len; k++) p[k] = z[k] + beta * p[k];
    s[PCG_RZ] = rz; s[PCG_RNORM2] = rr; s[PCG_ITERS]++;
  }
  return s[PCG_RNORM2] <= thr ? 1 : 0;
}
