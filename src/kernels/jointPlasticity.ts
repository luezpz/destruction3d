// Joint-chain plasticity and fracture (PLAN §4b, corner formulas of §2b). Pure typed-array kernel, no allocation per call.
//
// Per joint j (bond frame axes [n t1 t2]): total rotation phi: (3,) = [torsion, bend about t1, bend about t2] from body poses,
// plastic rest target p: (3,), elastic part e = phi - p, moments M = k o e = [Tq, M1, M2] (motor stiffness k: (3,), N m/rad;
// the spike showed k phi is the carried moment in quasi-static equilibrium). With N = V = 0, at face corner (y, z):
//   sigma_c = M1 z / I1 - M2 y / I2,  tau_c = |Tq| sqrt(y^2 + z^2) / J,  sigma_v = sqrt(sigma_c^2 + 3 tau_c^2).
// Plastic (sigmaY > 0): s_y = sigmaY + H eps_p; if max sigma_v > s_y, p += (1 - s_y/sigma_v) e (return to the yield moment),
//   eps_p += |dp| r_max / L (fibre strain of the curvature increment), break at eps_p > epsPlasticMax.
// Elastic (sigmaY == 0, epsElasticMax > 0): break at max|sigma_c| / E > epsElasticMax. Otherwise brittle: break at max sigma_c > sigmaT.

export interface PlasticViews {
  /** Bond-frame corner coords (y, z) per joint, CSR: corners of joint j are [cornerOffset[j], cornerOffset[j+1]) (2 doubles each). */
  cornerOffset: Uint32Array; // (m+1,)
  cornerYZ: Float64Array;
  I1: Float64Array; I2: Float64Array; J: Float64Array; // (m,) m^4
  /** Bond length (centre to centre, m) and max corner radius r_max (m): (m,). */
  length: Float64Array; rMax: Float64Array;
  /** Motor stiffness [torsion, bend1, bend2] (N m/rad): (3m,). */
  k: Float64Array;
  /** Material in Pa: sigmaY, H, E, sigmaT; strains epsPlasticMax, epsElasticMax: (m,) each. */
  sigmaY: Float64Array; H: Float64Array; E: Float64Array; sigmaT: Float64Array;
  epsPlasticMax: Float64Array; epsElasticMax: Float64Array;
  /** State: plastic target p (3m,), accumulated plastic strain (m,), 1 = joint active (m,). */
  target: Float64Array; epsP: Float64Array; alive: Uint8Array;
}

/**
 * Updates targets/strain from total rotations and flags changes and breaks.
 * @param phi (3m,) total joint rotation in the bond frame. @param changed (m,) out: 1 if the target moved (push to the motors).
 * @param broken (m,) out: 1 if the joint fails now (caller removes it and clears alive). @param ratio (m,) out: failure ratio (>= 1 at yield/break).
 */
export function updateJointPlasticity(v: PlasticViews, phi: Float64Array, changed: Uint8Array, broken: Uint8Array, ratio: Float64Array, count: number): void {
  for (let j = 0; j < count; j++) {
    changed[j] = 0; broken[j] = 0; ratio[j] = 0;
    if (!v.alive[j]) continue;
    const j3 = 3 * j;
    const e0 = phi[j3] - v.target[j3], e1 = phi[j3 + 1] - v.target[j3 + 1], e2 = phi[j3 + 2] - v.target[j3 + 2];
    const Tq = v.k[j3] * e0, M1 = v.k[j3 + 1] * e1, M2 = v.k[j3 + 2] * e2;
    let sv = 0, smax = 0, sabs = 0;
    for (let c = v.cornerOffset[j]; c < v.cornerOffset[j + 1]; c++) {
      const y = v.cornerYZ[2 * c], z = v.cornerYZ[2 * c + 1];
      const s = M1 * z / v.I1[j] - M2 * y / v.I2[j];
      const t = Math.abs(Tq) * Math.sqrt(y * y + z * z) / v.J[j];
      sv = Math.max(sv, Math.sqrt(s * s + 3 * t * t));
      smax = Math.max(smax, s);
      sabs = Math.max(sabs, Math.abs(s));
    }
    if (v.sigmaY[j] > 0) {
      const sy = v.sigmaY[j] + v.H[j] * v.epsP[j];
      ratio[j] = sv / sy;
      if (sv > sy) {
        const f = 1 - sy / sv, d0 = f * e0, d1 = f * e1, d2 = f * e2;
        v.target[j3] += d0; v.target[j3 + 1] += d1; v.target[j3 + 2] += d2;
        v.epsP[j] += Math.sqrt(d0 * d0 + d1 * d1 + d2 * d2) * v.rMax[j] / v.length[j];
        changed[j] = 1;
      }
      if (v.epsP[j] > v.epsPlasticMax[j]) broken[j] = 1;
    } else if (v.epsElasticMax[j] > 0) {
      ratio[j] = sabs / v.E[j] / v.epsElasticMax[j];
      if (ratio[j] > 1) broken[j] = 1;
    } else {
      ratio[j] = v.sigmaT[j] > 0 ? smax / v.sigmaT[j] : 0;
      if (ratio[j] > 1) broken[j] = 1;
    }
  }
}
