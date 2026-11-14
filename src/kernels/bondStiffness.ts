// Bond stiffness kernels (PLAN §2). Pure typed-array functions, no allocation per call.
//
// Per chunk u_i = [t_i(3), th_i(3)]. Face point motion of chunk i at the bond centroid c: t_i + th_i x r_i = t_i + X_i th_i,
// with r_i = c - x_i and X_i = -[r_i]x (3,3). S_i = [[I, X_i],[0, I]] (6,6).
// Face motion e = S_j u_j - S_i u_i : (6,). World stiffness D = R6 kd R6^T, R6 = blockdiag(R, R), R = [n t1 t2] (3,3),
// so D = blockdiag(Dt, Dr), Dt = R diag(kd0..2) R^T, Dr = R diag(kd3..5) R^T.
// K_b = T^T D T (12,12) with blocks K_ii = S_i^T D S_i, K_jj = S_j^T D S_j, K_ij = -S_i^T D S_j, K_ji = K_ij^T.
// kd = (1-d) * [EA/L, kGA/L, kGA/L, GJ/L, E I1/L, E I2/L], L = |r_i| + |r_j|, G = E/(2(1+nu)), k = 5/6.

/** Shear correction factor kappa for the bond shear springs. */
export const SHEAR_KAPPA = 5 / 6;
/** Stiffness floor factor so damaged bonds keep K SPD (PLAN §2). */
export const DAMAGE_FLOOR = 1e-3;
/** Doubles per bond in `bondK`: blocks ii, ij, jj (each 6x6 row-major). */
export const BOND_K_STRIDE = 108;

/** Read-only geometry/material views needed by the bond kernels (all from ChunkModel + material table). */
export interface BondViews {
  chunkPos: Float64Array; // (3n,)
  bondA: Uint32Array; // (m,)
  bondB: Uint32Array; // (m,)
  bondArea: Float64Array; // (m,)
  bondCentroid: Float64Array; // (3m,)
  bondNormal: Float64Array; // (3m,)
  bondT1: Float64Array; // (3m,)
  bondT2: Float64Array; // (3m,)
  bondI1: Float64Array; // (m,)
  bondI2: Float64Array; // (m,)
  bondJ: Float64Array; // (m,)
  bondMaterial: Uint16Array; // (m,)
  /** Per material id: E (GPa) and Poisson ratio. */
  matE: Float64Array;
  matNu: Float64Array;
}

// Module scratch (allocated once): R (3,3) row-major, Dt, Dr, X_a, X_b, temporaries.
const Rm = new Float64Array(9), Dt = new Float64Array(9), Dr = new Float64Array(9);
const Xa = new Float64Array(9), Xb = new Float64Array(9), DtXb = new Float64Array(9);

/** Writes X = -[r]x (3,3) row-major for r = c - x into `out`. */
function crossNeg(out: Float64Array, rx: number, ry: number, rz: number): void {
  out[0] = 0; out[1] = rz; out[2] = -ry;
  out[3] = -rz; out[4] = 0; out[5] = rx;
  out[6] = ry; out[7] = -rx; out[8] = 0;
}

/** out = R diag(k0,k1,k2) R^T (3,3); R columns are n, t1, t2 (from module scratch Rm). */
function rotDiag(out: Float64Array, k0: number, k1: number, k2: number): void {
  for (let a = 0; a < 3; a++)
    for (let b = 0; b < 3; b++)
      out[3 * a + b] = Rm[3 * a] * k0 * Rm[3 * b] + Rm[3 * a + 1] * k1 * Rm[3 * b + 1] + Rm[3 * a + 2] * k2 * Rm[3 * b + 2];
}

/**
 * Writes sign * S_a^T D S_b = sign * [[Dt, Dt Xb],[Xa^T Dt, Xa^T Dt Xb + Dr]] (6,6 row-major) at out[o..o+36).
 * Uses module scratch Dt, Dr, Xa, Xb.
 */
function writeBlock(out: Float64Array, o: number, A: Float64Array, B: Float64Array, sign: number): void {
  for (let a = 0; a < 3; a++)
    for (let b = 0; b < 3; b++) DtXb[3 * a + b] = Dt[3 * a] * B[b] + Dt[3 * a + 1] * B[3 + b] + Dt[3 * a + 2] * B[6 + b];
  for (let a = 0; a < 3; a++)
    for (let b = 0; b < 3; b++) {
      out[o + 6 * a + b] = sign * Dt[3 * a + b];
      out[o + 6 * a + 3 + b] = sign * DtXb[3 * a + b];
      // (Xa^T Dt)[a][b] = sum_k Xa[k][a] Dt[k][b]
      out[o + 6 * (3 + a) + b] = sign * (A[a] * Dt[b] + A[3 + a] * Dt[3 + b] + A[6 + a] * Dt[6 + b]);
      out[o + 6 * (3 + a) + 3 + b] = sign * (A[a] * DtXb[b] + A[3 + a] * DtXb[3 + b] + A[6 + a] * DtXb[6 + b] + Dr[3 * a + b]);
    }
}

/**
 * Computes per-bond local stiffness kd (6m,) and world blocks bondK (108m,: ii, ij, jj) for bonds [0, count).
 * @param damage (m,) d in [0,1); stiffness scaled by max(1-d, DAMAGE_FLOOR). @param alive (m,) 0 = removed (zero stiffness).
 */
export function computeBondStiffness(v: BondViews, damage: Float64Array, alive: Uint8Array, kd: Float64Array, bondK: Float64Array, count: number): void {
  const p = v.chunkPos, c = v.bondCentroid;
  for (let b = 0; b < count; b++) {
    const ia = 3 * v.bondA[b], ib = 3 * v.bondB[b], b3 = 3 * b, o = BOND_K_STRIDE * b;
    const rax = c[b3] - p[ia], ray = c[b3 + 1] - p[ia + 1], raz = c[b3 + 2] - p[ia + 2];
    const rbx = c[b3] - p[ib], rby = c[b3 + 1] - p[ib + 1], rbz = c[b3 + 2] - p[ib + 2];
    const L = Math.hypot(rax, ray, raz) + Math.hypot(rbx, rby, rbz);
    const mat = v.bondMaterial[b];
    const E = v.matE[mat] * 1e9, G = E / (2 * (1 + v.matNu[mat]));
    const s = alive[b] ? Math.max(1 - damage[b], DAMAGE_FLOOR) / L : 0;
    const A = v.bondArea[b];
    const k0 = s * E * A, k1 = s * SHEAR_KAPPA * G * A, k3 = s * G * v.bondJ[b], k4 = s * E * v.bondI1[b], k5 = s * E * v.bondI2[b];
    kd[6 * b] = k0; kd[6 * b + 1] = k1; kd[6 * b + 2] = k1; kd[6 * b + 3] = k3; kd[6 * b + 4] = k4; kd[6 * b + 5] = k5;
    for (let a = 0; a < 3; a++) {
      Rm[3 * a] = v.bondNormal[b3 + a]; Rm[3 * a + 1] = v.bondT1[b3 + a]; Rm[3 * a + 2] = v.bondT2[b3 + a];
    }
    rotDiag(Dt, k0, k1, k1);
    rotDiag(Dr, k3, k4, k5);
    crossNeg(Xa, rax, ray, raz);
    crossNeg(Xb, rbx, rby, rbz);
    writeBlock(bondK, o, Xa, Xa, 1);
    writeBlock(bondK, o + 36, Xa, Xb, -1);
    writeBlock(bondK, o + 72, Xb, Xb, 1);
  }
}

/**
 * Bond forces in the local frame F_b = kd * R6^T e = [N, V1, V2, Tq, M1, M2] (6m,) from full displacements u (6n,).
 * N > 0 = tension (n points a -> b). Supports simply have u = 0.
 */
export function computeBondForces(v: BondViews, kd: Float64Array, u: Float64Array, out: Float64Array, count: number): void {
  const p = v.chunkPos, c = v.bondCentroid;
  for (let b = 0; b < count; b++) {
    const ia = v.bondA[b], ib = v.bondB[b], b3 = 3 * b, ua = 6 * ia, ub = 6 * ib;
    const rax = c[b3] - p[3 * ia], ray = c[b3 + 1] - p[3 * ia + 1], raz = c[b3 + 2] - p[3 * ia + 2];
    const rbx = c[b3] - p[3 * ib], rby = c[b3 + 1] - p[3 * ib + 1], rbz = c[b3 + 2] - p[3 * ib + 2];
    // point motion t + th x r for each side
    const ax = u[ua] + u[ua + 4] * raz - u[ua + 5] * ray, ay = u[ua + 1] + u[ua + 5] * rax - u[ua + 3] * raz, az = u[ua + 2] + u[ua + 3] * ray - u[ua + 4] * rax;
    const bx = u[ub] + u[ub + 4] * rbz - u[ub + 5] * rby, by = u[ub + 1] + u[ub + 5] * rbx - u[ub + 3] * rbz, bz = u[ub + 2] + u[ub + 3] * rby - u[ub + 4] * rbx;
    const ex = bx - ax, ey = by - ay, ez = bz - az;
    const wx = u[ub + 3] - u[ua + 3], wy = u[ub + 4] - u[ua + 4], wz = u[ub + 5] - u[ua + 5];
    const n = v.bondNormal, t1 = v.bondT1, t2 = v.bondT2, k = 6 * b;
    out[k] = kd[k] * (n[b3] * ex + n[b3 + 1] * ey + n[b3 + 2] * ez);
    out[k + 1] = kd[k + 1] * (t1[b3] * ex + t1[b3 + 1] * ey + t1[b3 + 2] * ez);
    out[k + 2] = kd[k + 2] * (t2[b3] * ex + t2[b3 + 1] * ey + t2[b3 + 2] * ez);
    out[k + 3] = kd[k + 3] * (n[b3] * wx + n[b3 + 1] * wy + n[b3 + 2] * wz);
    out[k + 4] = kd[k + 4] * (t1[b3] * wx + t1[b3 + 1] * wy + t1[b3 + 2] * wz);
    out[k + 5] = kd[k + 5] * (t2[b3] * wx + t2[b3 + 1] * wy + t2[b3 + 2] * wz);
  }
}
