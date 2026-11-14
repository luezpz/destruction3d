// Debug bond lines: chunk A centre -> face centroid -> chunk B centre, coloured by failure ratio (green 0 -> yellow 0.5 -> red >= 1).

import * as THREE from 'three/webgpu';
import type { ChunkModel } from '../model/ChunkModel';

export interface BondLines {
  lines: THREE.LineSegments;
  colorAttr: THREE.BufferAttribute;
}

/** Builds 2 segments (4 vertices) per bond. */
export function createBondLines(model: ChunkModel): BondLines {
  const m = model.bondCount;
  const pos = new Float32Array(m * 12);
  for (let b = 0; b < m; b++) {
    const a = model.bondA[b], c = model.bondB[b];
    for (let k = 0; k < 3; k++) {
      const cen = model.bondCentroid[3 * b + k];
      pos[12 * b + k] = model.chunkPos[3 * a + k];
      pos[12 * b + 3 + k] = cen;
      pos[12 * b + 6 + k] = cen;
      pos[12 * b + 9 + k] = model.chunkPos[3 * c + k];
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const colorAttr = new THREE.BufferAttribute(new Float32Array(m * 12), 3).setUsage(THREE.DynamicDrawUsage);
  geo.setAttribute('color', colorAttr);
  const lines = new THREE.LineSegments(geo, new THREE.LineBasicNodeMaterial({ vertexColors: true, depthTest: false, transparent: true, blending: THREE.AdditiveBlending }));
  lines.renderOrder = 10;
  return { lines, colorAttr };
}

/**
 * Recolours bonds from a per-bond value (failure ratio or damage, 1 = failure). Lines blend additively, so black =
 * hidden: broken bonds and bonds below `minShow` disappear (declutters dense lattices).
 * @param ratio (m,) value per bond; @param broken (m,) 1 = broken; @param minShow value below which a bond is hidden.
 */
export function updateBondColors(bl: BondLines, ratio: ArrayLike<number>, broken: ArrayLike<number>, minShow = 0): void {
  const col = bl.colorAttr.array as Float32Array;
  for (let b = 0; b < ratio.length; b++) {
    const r = Math.min(Math.max(ratio[b], 0), 1);
    const hide = broken[b] || ratio[b] < minShow;
    const red = hide ? 0 : Math.min(1, 2 * r), green = hide ? 0 : Math.min(1, 2 - 2 * r);
    for (let v = 0; v < 4; v++) { col[12 * b + 3 * v] = red; col[12 * b + 3 * v + 1] = green; col[12 * b + 3 * v + 2] = 0; }
  }
  bl.colorAttr.needsUpdate = true;
}
