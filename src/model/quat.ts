// Small quaternion helpers (x, y, z, w; unit) for setup-time pose bookkeeping (island locals, demoted chunk poses). Headless.

/** Rotates v (3,) at v[vo..] by q (4,) at q[qo..]: q v q^-1 = v + w t + q_v x t, t = 2 q_v x v. Writes out[oo..oo+3). */
export function quatRotate(q: ArrayLike<number>, qo: number, v: ArrayLike<number>, vo: number, out: Float64Array | number[], oo = 0): void {
  const x = q[qo], y = q[qo + 1], z = q[qo + 2], w = q[qo + 3];
  const vx = v[vo], vy = v[vo + 1], vz = v[vo + 2];
  const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
  out[oo] = vx + w * tx + (y * tz - z * ty);
  out[oo + 1] = vy + w * ty + (z * tx - x * tz);
  out[oo + 2] = vz + w * tz + (x * ty - y * tx);
}

/** Quat of rotation vector th (3,) at th[o..] (axis * angle): (sin(|th|/2) th/|th|, cos(|th|/2)). Writes out[0..4). */
export function quatFromRotVec(th: ArrayLike<number>, o: number, out: Float64Array | number[]): void {
  const a = Math.hypot(th[o], th[o + 1], th[o + 2]), s = a > 1e-12 ? Math.sin(0.5 * a) / a : 0.5;
  out[0] = s * th[o]; out[1] = s * th[o + 1]; out[2] = s * th[o + 2]; out[3] = Math.cos(0.5 * a);
}

/** Hamilton product a b (apply b, then a) of quats at a[ao..], b[bo..] into out[oo..oo+4). */
export function quatMul(a: ArrayLike<number>, ao: number, b: ArrayLike<number>, bo: number, out: Float64Array | number[], oo = 0): void {
  const ax = a[ao], ay = a[ao + 1], az = a[ao + 2], aw = a[ao + 3];
  const bx = b[bo], by = b[bo + 1], bz = b[bo + 2], bw = b[bo + 3];
  out[oo] = aw * bx + ax * bw + ay * bz - az * by;
  out[oo + 1] = aw * by - ax * bz + ay * bw + az * bx;
  out[oo + 2] = aw * bz + ax * by - ay * bx + az * bw;
  out[oo + 3] = aw * bw - ax * bx - ay * by - az * bz;
}
