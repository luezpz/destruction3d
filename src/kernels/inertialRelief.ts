// D'Alembert inertial relief for free (unsupported) islands (PLAN §2): makes loads self-equilibrated so K u = f is consistent.
// Rigid acceleration a (3,), alpha (3,) from total force F and torque T about the COM c:
//   M a = F,  I alpha = T,  I = sum_i [m_i (|r_i|^2 I3 - r_i r_i^T) + j_i I3]  (3,3),  r_i = x_i - c.
// Then f_i -= m_i (a + alpha x r_i) (force part), f_i -= j_i alpha (moment part).

const s = new Float64Array(16); // scratch: c(3) F(3) T(3) I(6 sym: xx yy zz xy xz yz)

/**
 * In-place relief of f (6n,: force(3), moment(3) per chunk) for chunks [0, count).
 * @param mass (n,) kg. @param rotInertia (n,) isotropic own inertia j_i (kg m^2). @param pos (3n,) chunk centres.
 */
export function applyInertialRelief(f: Float64Array, mass: Float64Array, rotInertia: Float64Array, pos: Float64Array, count: number): void {
  s.fill(0);
  let M = 0;
  for (let i = 0; i < count; i++) {
    M += mass[i];
    for (let k = 0; k < 3; k++) s[k] += mass[i] * pos[3 * i + k];
  }
  if (!(M > 0)) return;
  for (let k = 0; k < 3; k++) s[k] /= M;
  for (let i = 0; i < count; i++) {
    const rx = pos[3 * i] - s[0], ry = pos[3 * i + 1] - s[1], rz = pos[3 * i + 2] - s[2], o = 6 * i, m = mass[i], j = rotInertia[i];
    const fx = f[o], fy = f[o + 1], fz = f[o + 2];
    s[3] += fx; s[4] += fy; s[5] += fz;
    s[6] += ry * fz - rz * fy + f[o + 3];
    s[7] += rz * fx - rx * fz + f[o + 4];
    s[8] += rx * fy - ry * fx + f[o + 5];
    const r2 = rx * rx + ry * ry + rz * rz;
    s[9] += m * (r2 - rx * rx) + j; s[10] += m * (r2 - ry * ry) + j; s[11] += m * (r2 - rz * rz) + j;
    s[12] -= m * rx * ry; s[13] -= m * rx * rz; s[14] -= m * ry * rz;
  }
  // alpha = I^-1 T by the adjugate (I is SPD unless a single massless-inertia chunk; then alpha = 0)
  const a = s[9], b = s[12], c = s[13], d = s[10], e = s[14], g = s[11];
  const c00 = d * g - e * e, c01 = c * e - b * g, c02 = b * e - c * d;
  const c11 = a * g - c * c, c12 = b * c - a * e, c22 = a * d - b * b;
  const det = a * c00 + b * c01 + c * c02;
  let ax = 0, ay = 0, az = 0;
  if (Math.abs(det) > 0) {
    ax = (c00 * s[6] + c01 * s[7] + c02 * s[8]) / det;
    ay = (c01 * s[6] + c11 * s[7] + c12 * s[8]) / det;
    az = (c02 * s[6] + c12 * s[7] + c22 * s[8]) / det;
  }
  const lx = s[3] / M, ly = s[4] / M, lz = s[5] / M;
  for (let i = 0; i < count; i++) {
    const rx = pos[3 * i] - s[0], ry = pos[3 * i + 1] - s[1], rz = pos[3 * i + 2] - s[2], o = 6 * i, m = mass[i], j = rotInertia[i];
    f[o] -= m * (lx + ay * rz - az * ry);
    f[o + 1] -= m * (ly + az * rx - ax * rz);
    f[o + 2] -= m * (lz + ax * ry - ay * rx);
    f[o + 3] -= j * ax; f[o + 4] -= j * ay; f[o + 5] -= j * az;
  }
}
