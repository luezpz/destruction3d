// Island mass properties from rest cell volumes (NOTES Design §3 step 2): hulls overlap, so Rapier's own per-hull
// volumes would double count; we compute mass, COM and principal inertia here and pass them explicitly.

/** Output of `islandMassProperties`. */
export interface IslandMass {
  mass: number; // kg
  com: [number, number, number]; // world (rest) COM
  /** Principal inertia (kg m^2) about the COM and the principal frame as quaternion (x,y,z,w) in world/rest axes. */
  inertia: [number, number, number];
  frame: [number, number, number, number];
}

/**
 * Mass properties of chunks[from..to): M = sum m_i, com = sum m_i x_i / M,
 * I = sum_i [ j_i I3 + m_i (|d_i|^2 I3 - d_i d_i^T) ] (3,3), d_i = x_i - com, then I = V diag(lambda) V^T (Jacobi).
 * @param chunks chunk ids. @param mass (n,) kg. @param rotInertia (n,) isotropic own inertia j_i. @param pos (3n,) rest centres.
 */
export function islandMassProperties(chunks: ArrayLike<number>, from: number, to: number, mass: Float64Array,
  rotInertia: Float64Array, pos: Float64Array): IslandMass {
  let M = 0, cx = 0, cy = 0, cz = 0;
  for (let k = from; k < to; k++) {
    const i = chunks[k], m = mass[i];
    M += m; cx += m * pos[3 * i]; cy += m * pos[3 * i + 1]; cz += m * pos[3 * i + 2];
  }
  cx /= M; cy /= M; cz /= M;
  const I = [0, 0, 0, 0, 0, 0, 0, 0, 0]; // row-major (3,3)
  for (let k = from; k < to; k++) {
    const i = chunks[k], m = mass[i];
    const d = [pos[3 * i] - cx, pos[3 * i + 1] - cy, pos[3 * i + 2] - cz];
    const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    for (let a = 0; a < 3; a++)
      for (let b = 0; b < 3; b++) I[3 * a + b] += (a === b ? rotInertia[i] + m * dd : 0) - m * d[a] * d[b];
  }
  const V = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  jacobiEigen3(I, V);
  // right-handed frame: flip the third axis if det(V) < 0
  const det = V[0] * (V[4] * V[8] - V[5] * V[7]) - V[1] * (V[3] * V[8] - V[5] * V[6]) + V[2] * (V[3] * V[7] - V[4] * V[6]);
  if (det < 0) { V[2] = -V[2]; V[5] = -V[5]; V[8] = -V[8]; }
  return { mass: M, com: [cx, cy, cz], inertia: [I[0], I[4], I[8]], frame: quatFromMatrix(V) };
}

/**
 * Cyclic Jacobi eigen-decomposition of symmetric A (3,3 row-major), in place: A -> diag(lambda), V accumulates
 * rotations so original A = V diag(lambda) V^T (V columns = eigenvectors).
 */
function jacobiEigen3(A: number[], V: number[]): void {
  for (let sweep = 0; sweep < 32; sweep++) {
    const off = A[1] * A[1] + A[2] * A[2] + A[5] * A[5];
    if (off <= 1e-30 * (A[0] * A[0] + A[4] * A[4] + A[8] * A[8]) || off === 0) return;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const apq = A[3 * p + q];
      if (apq === 0) continue;
      const theta = (A[3 * q + q] - A[3 * p + p]) / (2 * apq);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      // A <- G^T A G with Givens G in the (p,q) plane
      for (let k = 0; k < 3; k++) { // columns
        const akp = A[3 * k + p], akq = A[3 * k + q];
        A[3 * k + p] = c * akp - s * akq; A[3 * k + q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) { // rows
        const apk = A[3 * p + k], aqk = A[3 * q + k];
        A[3 * p + k] = c * apk - s * aqk; A[3 * q + k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const vkp = V[3 * k + p], vkq = V[3 * k + q];
        V[3 * k + p] = c * vkp - s * vkq; V[3 * k + q] = s * vkp + c * vkq;
      }
    }
  }
}

/** Unit quaternion (x,y,z,w) of a proper rotation matrix R (3,3 row-major), Shepperd's method. */
export function quatFromMatrix(R: number[]): [number, number, number, number] {
  const tr = R[0] + R[4] + R[8];
  if (tr > 0) {
    const s = 2 * Math.sqrt(tr + 1);
    return [(R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s, s / 4];
  }
  if (R[0] > R[4] && R[0] > R[8]) {
    const s = 2 * Math.sqrt(1 + R[0] - R[4] - R[8]);
    return [s / 4, (R[1] + R[3]) / s, (R[2] + R[6]) / s, (R[7] - R[5]) / s];
  }
  if (R[4] > R[8]) {
    const s = 2 * Math.sqrt(1 + R[4] - R[0] - R[8]);
    return [(R[1] + R[3]) / s, s / 4, (R[5] + R[7]) / s, (R[2] - R[6]) / s];
  }
  const s = 2 * Math.sqrt(1 + R[8] - R[0] - R[4]);
  return [(R[2] + R[6]) / s, (R[5] + R[7]) / s, s / 4, (R[3] - R[1]) / s];
}
