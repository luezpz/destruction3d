// Island body pose -> chunk skin poses, checked through the CPU skin reference.
import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { buildBondAdjacency } from '../src/kernels/bondAdjacency';
import { buildSkinWeights } from '../src/model/skinWeights';
import { POSE_STRIDE, skinVertices, writeIdentityPoses } from '../src/kernels/skin';
import { writeIslandPoses } from '../src/render/islandPose';

const t = createMaterialTable();
const m = generateFromBoxes(gridBoxes({ counts: [2, 1, 1], cell: [1, 1, 1], material: materialId(t, 'concrete') }), t);
const sw = buildSkinWeights(m, buildBondAdjacency(m.chunkCount, m.bondA, m.bondB, m.bondCount), new Uint8Array(m.bondCount).fill(1));
const V = sw.vertexCount;

function skin(body: number[], com: number[]) {
  const pose = new Float64Array(POSE_STRIDE * m.chunkCount), pos = new Float64Array(3 * V), nrm = new Float64Array(3 * V);
  writeIdentityPoses(pose, m.chunkCount);
  writeIslandPoses(pose, [1], body, 0, Float64Array.of(...[0, 1, 2].map((i) => m.chunkPos[3 + i] - com[i]), 0, 0, 0, 1), m.chunkPos);
  skinVertices(sw.entries, sw.vertOffset, sw.vertCount, sw.restPos, m.renderNormal, pose, pos, nrm, 0, V);
  return { pos, nrm };
}

describe('writeIslandPoses', () => {
  it('identity body pose at the COM reproduces rest positions', () => {
    const com = [1, 0.5, 0.5], { pos, nrm } = skin([...com, 0, 0, 0, 1], com);
    for (let i = 0; i < 3 * V; i++) {
      expect(pos[i]).toBeCloseTo(sw.restPos[i], 6);
      expect(nrm[i]).toBeCloseTo(m.renderNormal[i], 6);
    }
  });

  it('90 deg about z moves the chunk centre from +x to +y of the body', () => {
    const s = Math.SQRT1_2, { pos } = skin([0, 0, 0, 0, 0, s, s], [1, 0.5, 0.5]);
    let cx = 0, cy = 0;
    const v0 = m.renderVertOffset[1], v1 = m.renderVertOffset[2];
    for (let v = v0; v < v1; v++) { cx += pos[3 * v]; cy += pos[3 * v + 1]; }
    expect(cx / (v1 - v0)).toBeCloseTo(0, 5); // chunk 1 centre is +0.5 x from the COM -> +0.5 y
    expect(cy / (v1 - v0)).toBeCloseTo(0.5, 5);
  });
});
