// Debug/static chunk mesh: two meshes (outer + inner material) sharing one vertex buffer (PLAN §6).
// M1: vertices are baked at the rest pose; the pose texture + skinning replaces this in M4.

import * as THREE from 'three/webgpu';
import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import { CullBuffers, revealBond } from './cullBuffers';

export interface ChunkMeshes {
  outer: THREE.Mesh;
  inner: THREE.Mesh;
  /** Inner index attribute; appended slices are uploaded via addUpdateRange. */
  innerIndexAttr: THREE.BufferAttribute;
  cull: CullBuffers;
}

/** Hash a chunk id to a small brightness jitter so neighbouring chunks are distinguishable. */
function chunkTint(i: number): number {
  const h = Math.imul(i + 1, 2654435761) >>> 0;
  return 0.75 + 0.25 * ((h & 0xff) / 255);
}

/**
 * Builds the shared vertex buffer (world-space rest positions, normals, per-chunk debug colour) and both meshes.
 * @param cull culled index buffers from buildCullBuffers(model).
 */
export function createChunkMeshes(model: ChunkModel, table: MaterialTable, cull: CullBuffers): ChunkMeshes {
  const V = model.renderPos.length / 3;
  const pos = new Float32Array(3 * V), nrm = new Float32Array(model.renderNormal), col = new Float32Array(3 * V);
  const c = new THREE.Color();
  for (let i = 0; i < model.chunkCount; i++) {
    c.setHex(table.materials[model.chunkMaterial[i]].render.outer, THREE.SRGBColorSpace).multiplyScalar(chunkTint(i));
    for (let v = model.renderVertOffset[i]; v < model.renderVertOffset[i + 1]; v++) {
      for (let k = 0; k < 3; k++) pos[3 * v + k] = model.renderPos[3 * v + k] + model.chunkPos[3 * i + k];
      col[3 * v] = c.r; col[3 * v + 1] = c.g; col[3 * v + 2] = c.b;
    }
  }
  const position = new THREE.BufferAttribute(pos, 3), normal = new THREE.BufferAttribute(nrm, 3), color = new THREE.BufferAttribute(col, 3);

  const outerGeo = new THREE.BufferGeometry();
  outerGeo.setAttribute('position', position).setAttribute('normal', normal).setAttribute('color', color);
  outerGeo.setIndex(new THREE.BufferAttribute(cull.outerIndex, 1));

  const innerGeo = new THREE.BufferGeometry();
  innerGeo.setAttribute('position', position).setAttribute('normal', normal);
  // WebGPU rejects zero-size buffers: keep capacity >= 3 even when the model has no inner faces.
  const innerArr = cull.innerIndex.length ? cull.innerIndex : new Uint32Array(3);
  const innerIndexAttr = new THREE.BufferAttribute(innerArr, 1).setUsage(THREE.DynamicDrawUsage);
  innerGeo.setIndex(innerIndexAttr);
  innerGeo.setDrawRange(0, cull.innerCount);
  for (const g of [outerGeo, innerGeo]) g.computeBoundingSphere();

  const innerHex = table.materials[model.chunkMaterial[0] ?? 0]?.render.inner ?? 0x777777;
  const outer = new THREE.Mesh(outerGeo, new THREE.MeshStandardNodeMaterial({ vertexColors: true, roughness: 0.9 }));
  const inner = new THREE.Mesh(innerGeo, new THREE.MeshStandardNodeMaterial({ color: innerHex, roughness: 1 }));
  for (const m of [outer, inner]) { m.castShadow = true; m.receiveShadow = true; }
  return { outer, inner, innerIndexAttr, cull };
}

/** Reveals bond b's inner faces and uploads only the appended slice. @returns true if anything was appended. */
export function revealBondFaces(meshes: ChunkMeshes, b: number): boolean {
  const r = revealBond(meshes.cull, b);
  if (!r || r.count === 0) return false;
  meshes.innerIndexAttr.addUpdateRange(r.start, r.count);
  meshes.innerIndexAttr.needsUpdate = true;
  meshes.inner.geometry.setDrawRange(0, meshes.cull.innerCount);
  return true;
}
