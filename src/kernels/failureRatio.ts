// Failure ratio kernel (PLAN §2, §2b). Pure typed-array function, no allocation per call.
//
// Per bond b with local forces F_b = [N, V1, V2, Tq, M1, M2] (N > 0 tension), face frame [n t1 t2], centroid c:
// corner p, y = (p - c).t1, z = (p - c).t2, rho = |(y, z)|:
//   sigma = N/A + M1 z/I1 - M2 y/I2   (Pa, > 0 tension),   tau = |V|/A + |Tq| rho/J   (Pa), |V| = hypot(V1, V2).
// Ratio = max over enabled criteria (strengths scaled by the grain factor g = lerp(1, ratio_g, 1 - |n.axis|)):
//   tension sigma_max/sigmaT, compression -sigma_min/sigmaC, shear tau_max/(tauC + mu max(0, -N/A)),
//   plastic (sigmaY > 0): sigma_v/(sigmaY + H eps_p), sigma_v = max_corner sqrt(sigma^2 + 3 tau^2),
//   elastic (sigmaY == 0): (max_corner |sigma| / E) / epsElasticMax.

/** Per-material fields needed by the failure/damage kernels, packed by material id (see `materialViews`). */
export interface MaterialViews {
  E: Float64Array; // GPa
  sigmaT: Float64Array; // MPa
  sigmaC: Float64Array; // MPa (already resolved: 0 -> sigmaT)
  tauC: Float64Array; // MPa
  mu: Float64Array;
  sigmaY: Float64Array; // MPa
  H: Float64Array; // GPa
  epsElasticMax: Float64Array;
  damageRate: Float64Array; // 1/s per unit overload, may be Infinity
  filterTime: Float64Array; // s
  propagation: Float64Array;
  /** Grain axis (3k,) and strength ratio (k,); ratio 1 = isotropic. */
  grainAxis: Float64Array;
  grainRatio: Float64Array;
}

/** Bond geometry needed for corner stresses (subset of ChunkModel). */
export interface RatioGeomViews {
  bondArea: Float64Array; // (m,)
  bondCentroid: Float64Array; // (3m,)
  bondNormal: Float64Array; // (3m,)
  bondT1: Float64Array; // (3m,)
  bondT2: Float64Array; // (3m,)
  bondI1: Float64Array; // (m,)
  bondI2: Float64Array; // (m,)
  bondJ: Float64Array; // (m,)
  bondCornerOffset: Uint32Array; // (m+1,)
  bondCorners: Float64Array; // (3C,)
  bondMaterial: Uint16Array; // (m,)
}

/**
 * Writes the failure ratio (m,) of bonds [0, count) from local bond forces F (6m,). Dead bonds (alive = 0) get 0.
 * @param plasticStrain (m,) accumulated eps_p (0 until plasticity lands in M4); enters sigmaY + H eps_p.
 */
export function computeFailureRatios(g: RatioGeomViews, mat: MaterialViews, F: Float64Array, alive: Uint8Array,
  plasticStrain: Float64Array, out: Float64Array, count: number): void {
  for (let b = 0; b < count; b++) {
    if (!alive[b]) { out[b] = 0; continue; }
    const k = 6 * b, b3 = 3 * b, id = g.bondMaterial[b];
    const A = g.bondArea[b], I1 = g.bondI1[b], I2 = g.bondI2[b], J = g.bondJ[b];
    const N = F[k], V = Math.hypot(F[k + 1], F[k + 2]), Tq = Math.abs(F[k + 3]), M1 = F[k + 4], M2 = F[k + 5];
    const cx = g.bondCentroid[b3], cy = g.bondCentroid[b3 + 1], cz = g.bondCentroid[b3 + 2];
    let sMax = N / A, sMin = N / A, tMax = V / A, vMax = 0;
    for (let c = g.bondCornerOffset[b]; c < g.bondCornerOffset[b + 1]; c++) {
      const dx = g.bondCorners[3 * c] - cx, dy = g.bondCorners[3 * c + 1] - cy, dz = g.bondCorners[3 * c + 2] - cz;
      const y = dx * g.bondT1[b3] + dy * g.bondT1[b3 + 1] + dz * g.bondT1[b3 + 2];
      const z = dx * g.bondT2[b3] + dy * g.bondT2[b3 + 1] + dz * g.bondT2[b3 + 2];
      const s = N / A + (M1 * z) / I1 - (M2 * y) / I2;
      const t = V / A + (Tq * Math.hypot(y, z)) / J;
      if (s > sMax) sMax = s;
      if (s < sMin) sMin = s;
      if (t > tMax) tMax = t;
      const v = s * s + 3 * t * t;
      if (v > vMax) vMax = v;
    }
    const ax = mat.grainAxis[3 * id], ay = mat.grainAxis[3 * id + 1], az = mat.grainAxis[3 * id + 2];
    const cosA = Math.abs(g.bondNormal[b3] * ax + g.bondNormal[b3 + 1] * ay + g.bondNormal[b3 + 2] * az);
    const gf = 1 + (mat.grainRatio[id] - 1) * (1 - cosA); // grain factor on strengths (MPa -> Pa below)
    const st = gf * mat.sigmaT[id] * 1e6, sc = gf * mat.sigmaC[id] * 1e6;
    let r = Math.max(sMax / st, -sMin / sc, tMax / (gf * mat.tauC[id] * 1e6 + mat.mu[id] * Math.max(0, -N / A)));
    if (mat.sigmaY[id] > 0) {
      r = Math.max(r, Math.sqrt(vMax) / (gf * mat.sigmaY[id] * 1e6 + mat.H[id] * 1e9 * plasticStrain[b]));
    } else {
      r = Math.max(r, Math.max(sMax, -sMin) / (mat.E[id] * 1e9) / mat.epsElasticMax[id]);
    }
    out[b] = r;
  }
}
