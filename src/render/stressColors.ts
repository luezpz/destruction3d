// Debug heatmap: tints chunk vertices by the worst value (failure ratio or damage) over the chunk's bonds.

import * as THREE from 'three/webgpu';
import type { ChunkModel } from '../model/ChunkModel';

/** Heat colour for v in [0, 1]: blue 0 -> green 0.33 -> yellow 0.66 -> red 1. Writes rgb into out[o..o+3). */
export function heatColor(v: number, out: Float32Array, o: number): void {
  const t = Math.min(Math.max(v, 0), 1) * 3;
  const r = t < 1 ? 0 : t < 2 ? t - 1 : 1, g = t < 1 ? t : t < 2 ? 1 : 3 - t, b = t < 1 ? 1 - t : 0;
  out[o] = r; out[o + 1] = g; out[o + 2] = b;
}

/**
 * Writes per-vertex colours into `colorAttr`: heat of max bond value per chunk, or restores `base` when `value` is null.
 * @param value (m,) per-bond value (ratio or damage; 1 = failure) or null; @param base (3V,) original colours;
 * @param scratch (n,) per-chunk max (reused).
 */
export function colorChunksBy(model: ChunkModel, colorAttr: THREE.BufferAttribute, base: Float32Array, value: ArrayLike<number> | null, scratch: Float32Array): void {
  const col = colorAttr.array as Float32Array;
  if (!value) { col.set(base); colorAttr.needsUpdate = true; return; }
  scratch.fill(0);
  for (let b = 0; b < model.bondCount; b++) {
    const v = value[b];
    if (v > scratch[model.bondA[b]]) scratch[model.bondA[b]] = v;
    if (v > scratch[model.bondB[b]]) scratch[model.bondB[b]] = v;
  }
  for (let i = 0; i < model.chunkCount; i++) {
    const v0 = model.renderVertOffset[i], v1 = model.renderVertOffset[i + 1];
    heatColor(scratch[i], col, 3 * v0);
    for (let v = v0 + 1; v < v1; v++) { col[3 * v] = col[3 * v0]; col[3 * v + 1] = col[3 * v0 + 1]; col[3 * v + 2] = col[3 * v0 + 2]; }
  }
  colorAttr.needsUpdate = true;
}
