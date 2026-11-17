// CPU reference of the linear-blend skin (NOTES Design §4): same formulas as the TSL shader; used by tests and small low-poly parts.

/** Doubles per chunk in the pose buffer: a = x - q c (3), pad (1), q = (x, y, z, w) (4) — the 2 RGBA texels of the pose texture. */
export const POSE_STRIDE = 8;

/**
 * Writes chunk i's pose so a rest point p maps to q p + a, with a = x - q c (c = rest centre, x = current centre).
 * @param pose (8n,); @param x current centre (3,); @param q unit quaternion xyzw (4,); @param c rest centre (3,), each read at its offset.
 */
export function writeChunkPose(pose: Float64Array | Float32Array, i: number, x: ArrayLike<number>, xo: number, q: ArrayLike<number>, qo: number, c: ArrayLike<number>, co: number): void {
  const qx = q[qo], qy = q[qo + 1], qz = q[qo + 2], qw = q[qo + 3];
  const cx = c[co], cy = c[co + 1], cz = c[co + 2];
  // q c via t = 2 (q_v x c), q c = c + qw t + q_v x t
  const tx = 2 * (qy * cz - qz * cy), ty = 2 * (qz * cx - qx * cz), tz = 2 * (qx * cy - qy * cx);
  const rx = cx + qw * tx + (qy * tz - qz * ty), ry = cy + qw * ty + (qz * tx - qx * tz), rz = cz + qw * tz + (qx * ty - qy * tx);
  const o = POSE_STRIDE * i;
  pose[o] = x[xo] - rx; pose[o + 1] = x[xo + 1] - ry; pose[o + 2] = x[xo + 2] - rz; pose[o + 3] = 0;
  pose[o + 4] = qx; pose[o + 5] = qy; pose[o + 6] = qz; pose[o + 7] = qw;
}

/** Writes identity poses (q = 1, a = 0) for chunks [0, n). */
export function writeIdentityPoses(pose: Float64Array | Float32Array, n: number): void {
  for (let i = 0; i < n; i++) {
    const o = POSE_STRIDE * i;
    pose[o] = pose[o + 1] = pose[o + 2] = pose[o + 3] = 0;
    pose[o + 4] = pose[o + 5] = pose[o + 6] = 0; pose[o + 7] = 1;
  }
}

/**
 * Skins vertices [v0, v0 + vc): p' = sum_e w_e (q_e p + a_e), n' = normalize(sum_e w_e q_e n), entries `id << 16 | w16`.
 * @param restPos/restNormal (3V,) structure-frame rest data; @param pose (8n,) from writeChunkPose; @param outPos/outNormal (3V,). Alloc-free.
 */
export function skinVertices(
  entries: Uint32Array, vertOffset: Uint32Array, vertCount: Uint32Array,
  restPos: Float64Array, restNormal: Float64Array, pose: Float64Array | Float32Array,
  outPos: Float64Array, outNormal: Float64Array, v0: number, vc: number,
): void {
  for (let v = v0; v < v0 + vc; v++) {
    const px = restPos[3 * v], py = restPos[3 * v + 1], pz = restPos[3 * v + 2];
    const nx = restNormal[3 * v], ny = restNormal[3 * v + 1], nz = restNormal[3 * v + 2];
    let sx = 0, sy = 0, sz = 0, mx = 0, my = 0, mz = 0;
    const end = vertOffset[v] + vertCount[v];
    for (let e = vertOffset[v]; e < end; e++) {
      const en = entries[e], w = (en & 0xffff) / 65535, o = POSE_STRIDE * (en >>> 16);
      const qx = pose[o + 4], qy = pose[o + 5], qz = pose[o + 6], qw = pose[o + 7];
      // rotate p
      let tx = 2 * (qy * pz - qz * py), ty = 2 * (qz * px - qx * pz), tz = 2 * (qx * py - qy * px);
      sx += w * (px + qw * tx + (qy * tz - qz * ty) + pose[o]);
      sy += w * (py + qw * ty + (qz * tx - qx * tz) + pose[o + 1]);
      sz += w * (pz + qw * tz + (qx * ty - qy * tx) + pose[o + 2]);
      // rotate n
      tx = 2 * (qy * nz - qz * ny); ty = 2 * (qz * nx - qx * nz); tz = 2 * (qx * ny - qy * nx);
      mx += w * (nx + qw * tx + (qy * tz - qz * ty));
      my += w * (ny + qw * ty + (qz * tx - qx * tz));
      mz += w * (nz + qw * tz + (qx * ty - qy * tx));
    }
    const inv = 1 / (Math.hypot(mx, my, mz) || 1);
    outPos[3 * v] = sx; outPos[3 * v + 1] = sy; outPos[3 * v + 2] = sz;
    outNormal[3 * v] = mx * inv; outNormal[3 * v + 1] = my * inv; outNormal[3 * v + 2] = mz * inv;
  }
}
