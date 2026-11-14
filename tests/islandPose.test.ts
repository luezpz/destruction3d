import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { placeChunk } from '../src/render/islandPose';

const t = createMaterialTable();
const m = generateFromBoxes(gridBoxes({ counts: [2, 1, 1], cell: [1, 1, 1], material: materialId(t, 'concrete') }), t);

describe('placeChunk', () => {
  it('identity pose at the COM reproduces rest world positions', () => {
    const V = m.renderPos.length / 3, pos = new Float32Array(3 * V), nrm = new Float32Array(3 * V);
    const com = [1, 0.5, 0.5];
    placeChunk(m, 1, [...com, 0, 0, 0, 1], 0, com, pos, nrm);
    for (let v = m.renderVertOffset[1]; v < m.renderVertOffset[2]; v++)
      for (let k = 0; k < 3; k++) {
        expect(pos[3 * v + k]).toBeCloseTo(m.renderPos[3 * v + k] + m.chunkPos[3 + k], 6);
        expect(nrm[3 * v + k]).toBeCloseTo(m.renderNormal[3 * v + k], 6);
      }
  });

  it('90 deg about z moves the chunk centre from +x to +y of the body', () => {
    const V = m.renderPos.length / 3, pos = new Float32Array(3 * V), nrm = new Float32Array(3 * V);
    const s = Math.SQRT1_2, com = [1, 0.5, 0.5];
    placeChunk(m, 1, [0, 0, 0, 0, 0, s, s], 0, com, pos, nrm);
    let cx = 0, cy = 0;
    const v0 = m.renderVertOffset[1], v1 = m.renderVertOffset[2];
    for (let v = v0; v < v1; v++) { cx += pos[3 * v]; cy += pos[3 * v + 1]; }
    expect(cx / (v1 - v0)).toBeCloseTo(0, 5); // chunk 1 centre is +0.5 x from the COM -> +0.5 y
    expect(cy / (v1 - v0)).toBeCloseTo(0.5, 5);
  });
});
