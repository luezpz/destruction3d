// GPU vs CPU skin agreement (NOTES M4 tests): renders skinPositionNode output into a float target (one texel per vertex)
// and compares it with kernels/skin.ts. Opened by scripts/skinCheck.mjs; `?webgl` forces the WebGL2 fallback.

import * as THREE from 'three/webgpu';
import { float, normalLocal, positionLocal, varying, vec4, vertexIndex } from 'three/tsl';
import { POSE_STRIDE, skinVertices, writeChunkPose } from '../kernels/skin';
import { createSkinRangeAttribute, createSkinTextures, skinPositionNode, SkinTextures } from '../render/skinMaterial';
import type { SkinWeights } from '../model/skinWeights';

/** Deterministic PRNG (mulberry32) so failures reproduce. @param seed uint32. @returns () -> [0, 1). */
function rng(seed: number): () => number {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Synthetic weights: `perCount` vertices for each influence count in `counts`, ids among n chunks, w16 summing to 65535. */
function syntheticWeights(n: number, counts: number[], perCount: number, r: () => number): SkinWeights {
  const V = counts.length * perCount, vertOffset = new Uint32Array(V), vertCount = new Uint32Array(V);
  const list: number[] = [];
  let v = 0;
  for (const c of counts) for (let k = 0; k < perCount; k++, v++) {
    vertOffset[v] = list.length; vertCount[v] = c;
    const raw = Array.from({ length: c }, () => 0.05 + r()), s = raw.reduce((a, b) => a + b, 0);
    let left = 65535;
    for (let e = 0; e < c; e++) {
      const w = e === c - 1 ? left : Math.round((raw[e] / s) * 65535);
      left -= w;
      list.push(((Math.floor(r() * n) << 16) | w) >>> 0);
    }
  }
  return { vertexCount: V, vertOffset, vertCount, vertCapacity: vertCount.slice(), entries: Uint32Array.from(list) } as SkinWeights;
}

/** Offset added to encoded values on the GPU and removed after readback (|values| < 8 here); costs ~2e-6 of fp32 resolution. */
const ENC = 16;

/** Renders `node` (vec3 per vertex) into a V x 1 float target and reads it back. @returns (4V,) RGBA floats. */
async function renderPerVertex(renderer: THREE.WebGPURenderer, geo: THREE.BufferGeometry, tex: SkinTextures, color: any, V: number): Promise<Float32Array> {
  const mat = new THREE.PointsNodeMaterial();
  mat.positionNode = skinPositionNode(tex); // same hookup as chunkMesh: assigns positionLocal and normalLocal
  // place vertex v at texel v's centre: x_ndc = (v + 1/2) / V * 2 - 1
  mat.vertexNode = vec4(float(vertexIndex).add(0.5).div(V).mul(2).sub(1), 0, 0, 1);
  mat.colorNode = vec4(color.add(ENC), 1); // the output stage clamps negatives to 0: shift into positives
  const pts = new THREE.Points(geo, mat);
  pts.frustumCulled = false;
  const scene = new THREE.Scene(); scene.add(pts);
  const rt = new THREE.RenderTarget(V, 1, { type: THREE.FloatType, depthBuffer: false });
  renderer.setRenderTarget(rt);
  renderer.render(scene, new THREE.OrthographicCamera());
  renderer.setRenderTarget(null);
  const px = (await renderer.readRenderTargetPixelsAsync(rt, 0, 0, V, 1)) as Float32Array;
  rt.dispose(); mat.dispose();
  for (let i = 0; i < px.length; i++) if ((i & 3) !== 3) px[i] -= ENC;
  return px;
}

/** Runs the check. @returns per-count max |GPU - CPU| for positions and normals, plus the backend name. */
async function run() {
  const webgl = new URLSearchParams(location.search).has('webgl');
  const renderer = new THREE.WebGPURenderer({ forceWebGL: webgl });
  await renderer.init();
  const r = rng(12345), n = 64, counts = [1, 8, 32], perCount = 256;
  const sw = syntheticWeights(n, counts, perCount, r), V = sw.vertexCount;

  // random rigid poses: centres in [-2, 2]^3 moved by up to 1, random unit quaternions
  const tex = createSkinTextures(n, sw), pose64 = new Float64Array(POSE_STRIDE * n);
  for (let i = 0; i < n; i++) {
    const c = [4 * r() - 2, 4 * r() - 2, 4 * r() - 2], x = c.map((v) => v + 2 * r() - 1);
    const q = [r() - 0.5, r() - 0.5, r() - 0.5, r() - 0.5], ql = Math.hypot(...q);
    writeChunkPose(pose64, i, x, 0, q.map((v) => v / ql), 0, c, 0);
  }
  tex.pose.set(pose64); // GPU and CPU both read the Float32 values
  const pose32 = tex.pose.subarray(0, POSE_STRIDE * n);

  const restPos = new Float64Array(3 * V), restNormal = new Float64Array(3 * V);
  for (let v = 0; v < V; v++) {
    for (let k = 0; k < 3; k++) restPos[3 * v + k] = 4 * r() - 2;
    const nn = [r() - 0.5, r() - 0.5, r() - 0.5], l = Math.hypot(...nn);
    for (let k = 0; k < 3; k++) restNormal[3 * v + k] = nn[k] / l;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(restPos), 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(Float32Array.from(restNormal), 3));
  geo.setAttribute('skinRange', createSkinRangeAttribute(sw));

  const outPos = new Float64Array(3 * V), outNormal = new Float64Array(3 * V);
  skinVertices(sw.entries, sw.vertOffset, sw.vertCount, Float64Array.from(Float32Array.from(restPos)), Float64Array.from(Float32Array.from(restNormal)), pose32, outPos, outNormal, 0, V);

  const gpuPos = await renderPerVertex(renderer, geo, tex, varying(positionLocal), V);
  const gpuNormal = await renderPerVertex(renderer, geo, tex, varying(normalLocal), V);

  const result: Record<string, { pos: number; normal: number }> = {};
  counts.forEach((c, ci) => {
    let dp = 0, dn = 0;
    for (let v = ci * perCount; v < (ci + 1) * perCount; v++) for (let k = 0; k < 3; k++) {
      dp = Math.max(dp, Math.abs(gpuPos[4 * v + k] - outPos[3 * v + k]));
      dn = Math.max(dn, Math.abs(gpuNormal[4 * v + k] - outNormal[3 * v + k]));
    }
    result[c] = { pos: dp, normal: dn };
  });
  return { backend: webgl ? 'webgl2' : 'webgpu', isWebGPU: (renderer.backend as any).isWebGPUBackend === true, result };
}

const out = document.getElementById('out')!;
const done = run().then((r) => { out.textContent = JSON.stringify(r, null, 1); return r; }, (e) => { out.textContent = String(e?.stack ?? e); return { error: String(e?.stack ?? e) }; });
(window as any).skinCheck = done;
