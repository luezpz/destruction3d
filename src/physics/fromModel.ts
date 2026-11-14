// Float64 ChunkModel -> Float32 Rapier boundary conversions.

import type { ChunkModel } from '../model/ChunkModel';
import type { IslandChunk } from './PhysicsBackend';

/**
 * Collider input for chunk i: its cached hull points (chunk-local, Float32) and triangle indices.
 * @param origin body origin in world space; translation = chunk rest pos - origin (keeps f32 values small).
 */
export function chunkHull(model: ChunkModel, i: number, origin: [number, number, number] = [0, 0, 0]): IslandChunk {
  const p0 = model.hullPointOffset[i], p1 = model.hullPointOffset[i + 1];
  const t0 = model.hullTriOffset[i], t1 = model.hullTriOffset[i + 1];
  return {
    chunkId: i,
    points: new Float32Array(model.hullPoints.subarray(3 * p0, 3 * p1)),
    indices: new Uint32Array(model.hullTris.subarray(3 * t0, 3 * t1)),
    translation: [
      model.chunkPos[3 * i] - origin[0],
      model.chunkPos[3 * i + 1] - origin[1],
      model.chunkPos[3 * i + 2] - origin[2],
    ],
  };
}
