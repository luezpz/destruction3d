// CPU rigid placement of a chunk's render vertices (debug path until M4's pose texture + TSL skinning).
// Headless (no three.js). World vertex p = x_b + q_b (r_c + v), normal n' = q_b n, where x_b, q_b = body pose,
// r_c = chunk rest pos - island COM (body-local), v / n = chunk-local rest vertex / normal.

import type { ChunkModel } from '../model/ChunkModel';

/** Rotates (x, y, z) by unit quaternion q = (qx, qy, qz, qw) at q[o..o+4) and writes to out[k..k+3). */
function rotate(q: ArrayLike<number>, o: number, x: number, y: number, z: number, out: Float32Array, k: number, tx: number, ty: number, tz: number): void {
  const qx = q[o], qy = q[o + 1], qz = q[o + 2], qw = q[o + 3];
  // v' = v + 2 qw (u x v) + 2 u x (u x v), u = (qx, qy, qz)
  const cx = 2 * (qy * z - qz * y), cy = 2 * (qz * x - qx * z), cz = 2 * (qx * y - qy * x);
  out[k] = tx + x + qw * cx + (qy * cz - qz * cy);
  out[k + 1] = ty + y + qw * cy + (qz * cx - qx * cz);
  out[k + 2] = tz + z + qw * cz + (qx * cy - qy * cx);
}

/**
 * Writes chunk c's vertices/normals into pos/nrm (3V,) for body pose (x, y, z, qx, qy, qz, qw) at pose[o..o+7)
 * and island COM `com` (rest, world). Float64 rest data -> Float32 GPU buffers here only.
 */
export function placeChunk(model: ChunkModel, c: number, pose: ArrayLike<number>, o: number, com: ArrayLike<number>,
  pos: Float32Array, nrm: Float32Array): void {
  const rx = model.chunkPos[3 * c] - com[0], ry = model.chunkPos[3 * c + 1] - com[1], rz = model.chunkPos[3 * c + 2] - com[2];
  for (let v = model.renderVertOffset[c]; v < model.renderVertOffset[c + 1]; v++) {
    const k = 3 * v;
    rotate(pose, o + 3, rx + model.renderPos[k], ry + model.renderPos[k + 1], rz + model.renderPos[k + 2], pos, k, pose[o], pose[o + 1], pose[o + 2]);
    rotate(pose, o + 3, model.renderNormal[k], model.renderNormal[k + 1], model.renderNormal[k + 2], nrm, k, 0, 0, 0);
  }
}
