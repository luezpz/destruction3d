// Explicit 6-axis joint springs between dynamic bodies (NOTES Design §4b, option a: Rapier JS has no per-axis motors).
// Pure typed-array kernel, no allocation per call.
//
// Per joint j between bodies a, b with body-local anchors r_a, r_b (3,) (both at the bond centroid at rest),
// bond frame B_j (3,3, columns [n t1 t2]) expressed in body a's frame, rest relative rotation q0 = q_a0^-1 q_b0.
// Error in the bond frame: dx = B^T R_a^T (p_b - p_a), p = x + R r (anchor gap, rest = 0 + plastic d0),
//                          phi = B^T log(q0^-1 q_a^-1 q_b) (rotation vector, small-angle 2 vec(q) with w >= 0).
// Damped rates use relative velocities in the same frame. Local wrench f = -(k o e + c o edot) : (6,).
// World force on b: F = R_a B f_t; on a: -F. Torque on b: r_b,w x F + R_a B f_r; on a: -(r_a,w x F) - R_a B f_r
// where r_w = R r (world lever). Sign convention N > 0 = tension like kernels/bondStiffness.ts (f_t = -k dx, so
// N = k dx[0]).

/** Doubles per body in poses: x(3) q(4: x,y,z,w) v(3) w(3). */
export const BODY_STATE_STRIDE = 13;

export interface JointViews {
  jointA: Uint32Array; // (m,) body index of side a
  jointB: Uint32Array; // (m,)
  anchorA: Float64Array; // (3m,) body-local anchor on a
  anchorB: Float64Array; // (3m,)
  frame: Float64Array; // (9m,) bond frame B in body a's frame, row-major, columns [n t1 t2]
  restQ: Float64Array; // (4m,) q0 = q_a^-1 q_b at rest (plastic target), x,y,z,w
  restD: Float64Array; // (3m,) plastic translational offset d0 in the bond frame
  k: Float64Array; // (6m,) capped stiffness [EA/L, kGA/L, kGA/L, GJ/L, EI1/L, EI2/L] (N/m, N m/rad)
  c: Float64Array; // (6m,) damping per axis (N s/m, N m s/rad)
  alive: Uint8Array; // (m,)
}

// Module scratch: rotation matrices of a and b (3,3 row-major), world bond frame W = R_a B.
const Ra = new Float64Array(9), Rb = new Float64Array(9), W = new Float64Array(9);

/** Writes the row-major rotation matrix of unit quaternion (x,y,z,w) at q[o..o+4) into out (3,3). */
export function quatToMat(q: ArrayLike<number>, o: number, out: Float64Array): void {
  const x = q[o], y = q[o + 1], z = q[o + 2], w = q[o + 3];
  out[0] = 1 - 2 * (y * y + z * z); out[1] = 2 * (x * y - z * w); out[2] = 2 * (x * z + y * w);
  out[3] = 2 * (x * y + z * w); out[4] = 1 - 2 * (x * x + z * z); out[5] = 2 * (y * z - x * w);
  out[6] = 2 * (x * z - y * w); out[7] = 2 * (y * z + x * w); out[8] = 1 - 2 * (x * x + y * y);
}

/**
 * Computes joint errors e (6m,: dx(3), phi(3) in the bond frame) and accumulates spring-damper wrenches into
 * `wrench` (6 per body: force(3), torque(3), world frame; caller zeroes it). Removed joints are skipped.
 * @param state (13 nb,) body states (BODY_STATE_STRIDE). @param err (6m,) out errors. @param force (6m,) out local wrench f.
 */
export function computeJointSprings(v: JointViews, state: Float64Array, err: Float64Array, force: Float64Array, wrench: Float64Array, count: number): void {
  for (let j = 0; j < count; j++) {
    const j6 = 6 * j;
    if (!v.alive[j]) { for (let i = 0; i < 6; i++) { err[j6 + i] = 0; force[j6 + i] = 0; } continue; }
    const a = v.jointA[j], b = v.jointB[j], sa = BODY_STATE_STRIDE * a, sb = BODY_STATE_STRIDE * b, j3 = 3 * j, j9 = 9 * j;
    quatToMat(state, sa + 3, Ra);
    quatToMat(state, sb + 3, Rb);
    // W = R_a B (3,3)
    for (let r = 0; r < 3; r++)
      for (let cc = 0; cc < 3; cc++)
        W[3 * r + cc] = Ra[3 * r] * v.frame[j9 + cc] + Ra[3 * r + 1] * v.frame[j9 + 3 + cc] + Ra[3 * r + 2] * v.frame[j9 + 6 + cc];
    // world levers r_w = R r
    const ax = v.anchorA[j3], ay = v.anchorA[j3 + 1], az = v.anchorA[j3 + 2];
    const bx = v.anchorB[j3], by = v.anchorB[j3 + 1], bz = v.anchorB[j3 + 2];
    const rax = Ra[0] * ax + Ra[1] * ay + Ra[2] * az, ray = Ra[3] * ax + Ra[4] * ay + Ra[5] * az, raz = Ra[6] * ax + Ra[7] * ay + Ra[8] * az;
    const rbx = Rb[0] * bx + Rb[1] * by + Rb[2] * bz, rby = Rb[3] * bx + Rb[4] * by + Rb[5] * bz, rbz = Rb[6] * bx + Rb[7] * by + Rb[8] * bz;
    // anchor gap and its rate (v + w x r)
    const gx = state[sb] + rbx - state[sa] - rax, gy = state[sb + 1] + rby - state[sa + 1] - ray, gz = state[sb + 2] + rbz - state[sa + 2] - raz;
    const wax = state[sa + 10], way = state[sa + 11], waz = state[sa + 12], wbx = state[sb + 10], wby = state[sb + 11], wbz = state[sb + 12];
    const vgx = state[sb + 7] + wby * rbz - wbz * rby - state[sa + 7] - (way * raz - waz * ray);
    const vgy = state[sb + 8] + wbz * rbx - wbx * rbz - state[sa + 8] - (waz * rax - wax * raz);
    const vgz = state[sb + 9] + wbx * rby - wby * rbx - state[sa + 9] - (wax * ray - way * rax);
    // relative rotation qe = q0^-1 qa^-1 qb; rotation vector ~ 2 sign(w) vec(qe) in a's frame, then into the bond frame
    const qax = -state[sa + 3], qay = -state[sa + 4], qaz = -state[sa + 5], qaw = state[sa + 6]; // qa^-1
    const qbx = state[sb + 3], qby = state[sb + 4], qbz = state[sb + 5], qbw = state[sb + 6];
    const rx = qaw * qbx + qax * qbw + qay * qbz - qaz * qby, ry = qaw * qby - qax * qbz + qay * qbw + qaz * qbx;
    const rz = qaw * qbz + qax * qby - qay * qbx + qaz * qbw, rw = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
    const ox = -v.restQ[4 * j], oy = -v.restQ[4 * j + 1], oz = -v.restQ[4 * j + 2], ow = v.restQ[4 * j + 3]; // q0^-1
    let ex = ow * rx + ox * rw + oy * rz - oz * ry, ey = ow * ry - ox * rz + oy * rw + oz * rx;
    let ez = ow * rz + ox * ry - oy * rx + oz * rw;
    const ew = ow * rw - ox * rx - oy * ry - oz * rz;
    const s = ew < 0 ? -2 : 2; ex *= s; ey *= s; ez *= s; // phi in a's body frame
    const F = v.frame, k = v.k, c = v.c;
    for (let i = 0; i < 3; i++) {
      // bond axis i in world = column i of W; in a's frame = column i of B
      const wi0 = W[i], wi1 = W[3 + i], wi2 = W[6 + i];
      const dx = wi0 * gx + wi1 * gy + wi2 * gz - v.restD[j3 + i];
      const dv = wi0 * vgx + wi1 * vgy + wi2 * vgz;
      const ph = F[j9 + i] * ex + F[j9 + 3 + i] * ey + F[j9 + 6 + i] * ez;
      const dw = wi0 * (wbx - wax) + wi1 * (wby - way) + wi2 * (wbz - waz);
      err[j6 + i] = dx; err[j6 + 3 + i] = ph;
      force[j6 + i] = -(k[j6 + i] * dx + c[j6 + i] * dv);
      force[j6 + 3 + i] = -(k[j6 + 3 + i] * ph + c[j6 + 3 + i] * dw);
    }
    // world force/torque on b
    const Fx = W[0] * force[j6] + W[1] * force[j6 + 1] + W[2] * force[j6 + 2];
    const Fy = W[3] * force[j6] + W[4] * force[j6 + 1] + W[5] * force[j6 + 2];
    const Fz = W[6] * force[j6] + W[7] * force[j6 + 1] + W[8] * force[j6 + 2];
    const Tx = W[0] * force[j6 + 3] + W[1] * force[j6 + 4] + W[2] * force[j6 + 5];
    const Ty = W[3] * force[j6 + 3] + W[4] * force[j6 + 4] + W[5] * force[j6 + 5];
    const Tz = W[6] * force[j6 + 3] + W[7] * force[j6 + 4] + W[8] * force[j6 + 5];
    const ob = 6 * b, oa = 6 * a;
    wrench[ob] += Fx; wrench[ob + 1] += Fy; wrench[ob + 2] += Fz;
    wrench[ob + 3] += rby * Fz - rbz * Fy + Tx; wrench[ob + 4] += rbz * Fx - rbx * Fz + Ty; wrench[ob + 5] += rbx * Fy - rby * Fx + Tz;
    wrench[oa] -= Fx; wrench[oa + 1] -= Fy; wrench[oa + 2] -= Fz;
    wrench[oa + 3] -= ray * Fz - raz * Fy + Tx; wrench[oa + 4] -= raz * Fx - rax * Fz + Ty; wrench[oa + 5] -= rax * Fy - ray * Fx + Tz;
  }
}

/**
 * Caps stiffness for explicit stability (NOTES Design §4b): k <= m_eff omega^2 (translation), I_eff omega^2 (rotation),
 * omegaMax (rad/s) ~ 1/(2-4 dt). Damping c = 2 zeta sqrt(k m) (zeta = fraction of critical).
 * A translational spring at lever l also acts as a rotational spring k l^2 on the bodies, so translation uses
 * m = min(m_eff, I_eff / l^2) (without this the chain spins up, spike 2026-11-14).
 * @param mEff (m,) reduced mass (kg); iEff (m,) reduced inertia (kg m^2, isotropic estimate); lever (m,) max anchor lever (m).
 */
export function capJointStiffness(kRaw: Float64Array, mEff: Float64Array, iEff: Float64Array, lever: Float64Array, omegaMax: number, zeta: number, k: Float64Array, c: Float64Array, count: number): void {
  const w2 = omegaMax * omegaMax;
  for (let j = 0; j < count; j++) {
    const mt = Math.min(mEff[j], iEff[j] / Math.max(lever[j] * lever[j], 1e-12));
    for (let i = 0; i < 6; i++) {
      const m = i < 3 ? mt : iEff[j];
      const kk = Math.min(kRaw[6 * j + i], m * w2);
      k[6 * j + i] = kk;
      c[6 * j + i] = 2 * zeta * Math.sqrt(kk * m);
    }
  }
}
