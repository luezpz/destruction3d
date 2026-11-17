// Chunk poses of a rigid island from its body pose, written into the skin pose buffer (NOTES Design §4 "Pose per chunk"). Headless.
// Chunk pose = body * local: x_c = x_b + q_b t, q_c = q_b r (t, r = chunk pose in the body frame, Island.local).

import { writeChunkPose } from '../kernels/skin';
import { quatMul, quatRotate } from '../model/quat';

const x = new Float64Array(3), q = new Float64Array(4);

/**
 * Writes the skin pose of each chunk in `chunks` for body pose (x, y, z, qx, qy, qz, qw) at body[o..o+7).
 * @param pose (8n,) skin pose buffer. @param local (7k,) chunk poses in the body frame (same order as chunks).
 * @param restPos (3n,) rest chunk centres (ChunkModel.chunkPos).
 */
export function writeIslandPoses(pose: Float32Array | Float64Array, chunks: ArrayLike<number>, body: ArrayLike<number>, o: number,
  local: Float64Array, restPos: Float64Array): void {
  for (let k = 0; k < chunks.length; k++) {
    quatRotate(body, o + 3, local, 7 * k, x);
    x[0] += body[o]; x[1] += body[o + 1]; x[2] += body[o + 2];
    quatMul(body, o + 3, local, 7 * k + 3, q);
    writeChunkPose(pose, chunks[k], x, 0, q, 0, restPos, 3 * chunks[k]);
  }
}
