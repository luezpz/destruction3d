// GPU skin (PLAN §4 "Variable-length weights", "Delivery"): pose + entry data textures read with textureLoad in a TSL positionNode.
// Data textures (not storage buffers) so the WebGL2 fallback works unchanged. Mirrors kernels/skin.ts (CPU reference).

import * as THREE from 'three/webgpu';
import { attribute, float, Fn, int, ivec2, Loop, normalLocal, textureLoad, uint, vec3 } from 'three/tsl';
import { POSE_STRIDE } from '../kernels/skin';
import type { SkinWeights } from '../model/skinWeights';

/** Texture row width in texels for both data textures. */
export const SKIN_TEX_WIDTH = 2048;

export interface SkinTextures {
  /** RGBA32F, 2 texels/chunk: [a.xyz, 0], q.xyzw. `pose` is its backing array (write with writeChunkPose / writeIslandPoses). */
  poseTex: THREE.DataTexture;
  pose: Float32Array;
  /** R32UI, one entry per texel (`id << 16 | w16`). */
  entryTex: THREE.DataTexture;
  entries: Uint32Array;
}

/** Rows needed for `texels` texels at SKIN_TEX_WIDTH (>= 1). */
function rows(texels: number): number {
  return Math.max(1, Math.ceil(texels / SKIN_TEX_WIDTH));
}

/** Allocates both textures for n chunks and the weights' entries (copied). Poses start uninitialized: write identity first. */
export function createSkinTextures(n: number, sw: SkinWeights): SkinTextures {
  const ph = rows((POSE_STRIDE / 4) * n), pose = new Float32Array(SKIN_TEX_WIDTH * ph * 4);
  const poseTex = new THREE.DataTexture(pose, SKIN_TEX_WIDTH, ph, THREE.RGBAFormat, THREE.FloatType);
  const eh = rows(sw.entries.length), entries = new Uint32Array(SKIN_TEX_WIDTH * eh);
  entries.set(sw.entries);
  const entryTex = new THREE.DataTexture(entries, SKIN_TEX_WIDTH, eh, THREE.RedIntegerFormat, THREE.UnsignedIntType);
  for (const t of [poseTex, entryTex]) { t.magFilter = t.minFilter = THREE.NearestFilter; t.generateMipmaps = false; t.needsUpdate = true; }
  return { poseTex, pose, entryTex, entries };
}

/** Copies the dirty vertices' slots (after reweightBroken) into the entry texture and flags it for upload. */
export function uploadDirtyWeights(tex: SkinTextures, sw: SkinWeights): void {
  if (!sw.dirtyCount) return;
  for (let i = 0; i < sw.dirtyCount; i++) {
    const v = sw.dirtyVerts[i], o = sw.vertOffset[v];
    for (let e = o; e < o + sw.vertCount[v]; e++) tex.entries[e] = sw.entries[e];
  }
  tex.entryTex.needsUpdate = true; // whole-texture upload; range uploads if profiling says so
}

/** Per-vertex `skinRange` attribute uvec2 (offset, count) aliasing the weights arrays' current counts. Re-flag needsUpdate after reweights. */
export function createSkinRangeAttribute(sw: SkinWeights): THREE.BufferAttribute {
  const r = new Uint32Array(2 * sw.vertexCount);
  writeSkinRanges(r, sw);
  return new THREE.BufferAttribute(r, 2).setUsage(THREE.DynamicDrawUsage);
}

/** Writes (offset, count) per vertex into r (2V,). */
export function writeSkinRanges(r: Uint32Array, sw: SkinWeights): void {
  for (let v = 0; v < sw.vertexCount; v++) { r[2 * v] = sw.vertOffset[v]; r[2 * v + 1] = sw.vertCount[v]; }
}

/**
 * TSL position node: p' = sum_e w_e (q_e p + a_e) over the vertex's entry range; also assigns normalLocal = normalize(sum w_e q_e n).
 * Expects the geometry `position` = structure-frame rest positions and attribute `skinRange`.
 */
export function skinPositionNode(tex: SkinTextures) {
  return Fn(() => {
    const range = attribute('skinRange', 'uvec2');
    const p = attribute('position', 'vec3'), n = attribute('normal', 'vec3');
    const sp = vec3(0).toVar(), sn = vec3(0).toVar();
    const W = uint(SKIN_TEX_WIDTH);
    Loop({ start: range.x, end: range.x.add(range.y), type: 'uint', condition: '<' }, ({ i }: { i: any }) => {
      const e = uint(textureLoad(tex.entryTex, ivec2(int(i.mod(W)), int(i.div(W)))).x).toVar();
      const id = e.shiftRight(uint(16));
      const w = float(e.bitAnd(uint(0xffff))).div(65535);
      const t0 = id.mul(uint(2));
      const aT = textureLoad(tex.poseTex, ivec2(int(t0.mod(W)), int(t0.div(W))));
      const t1 = t0.add(uint(1));
      const q = textureLoad(tex.poseTex, ivec2(int(t1.mod(W)), int(t1.div(W))));
      // rotate by q: v + qw t + q_v x t, t = 2 q_v x v
      const tp = q.xyz.cross(p).mul(2), tn = q.xyz.cross(n).mul(2);
      sp.addAssign(p.add(tp.mul(q.w)).add(q.xyz.cross(tp)).add(aT.xyz).mul(w));
      sn.addAssign(n.add(tn.mul(q.w)).add(q.xyz.cross(tn)).mul(w));
    });
    normalLocal.assign(sn.normalize());
    return sp;
  })();
}
