import { describe, expect, it } from 'vitest';
import { ChunkFlag, ChunkModel, OUTER_FACE, surfaceChunkCount } from '../src/model/ChunkModel';
import { MATERIAL_PRESETS, createMaterialTable, interfaceMaterial, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { brickWallBoxes, gridBoxes } from '../src/model/generators';

/** Signed volume of chunk i's closed hull via the divergence theorem (sum of origin tets). */
function hullVolume(m: ChunkModel, i: number): number {
  const p = m.hullPoints, base = 3 * m.hullPointOffset[i];
  let v = 0;
  for (let t = m.hullTriOffset[i]; t < m.hullTriOffset[i + 1]; t++) {
    const [a, b, c] = [0, 1, 2].map((k) => base + 3 * m.hullTris[3 * t + k]);
    v += p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1])
      - p[a + 1] * (p[b] * p[c + 2] - p[b + 2] * p[c])
      + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c]);
  }
  return v / 6;
}

/** Sum of render triangle areas, split by outer / inner. Also checks normals point along the triangle winding. */
function renderAreas(m: ChunkModel): { outer: number; inner: number } {
  let outer = 0, inner = 0;
  for (let t = 0; t < m.renderTriTag.length; t++) {
    const [a, b, c] = [0, 1, 2].map((k) => 3 * m.renderTris[3 * t + k]);
    const p = m.renderPos;
    const u = [p[b] - p[a], p[b + 1] - p[a + 1], p[b + 2] - p[a + 2]];
    const w = [p[c] - p[a], p[c + 1] - p[a + 1], p[c + 2] - p[a + 2]];
    const cr = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
    const dot = cr[0] * m.renderNormal[a] + cr[1] * m.renderNormal[a + 1] + cr[2] * m.renderNormal[a + 2];
    expect(dot).toBeGreaterThan(0); // CCW seen from outside
    const area = 0.5 * Math.hypot(cr[0], cr[1], cr[2]);
    if (m.renderTriTag[t] === OUTER_FACE) outer += area; else inner += area;
  }
  return { outer, inner };
}

describe('materials', () => {
  it('presets load and validate', () => {
    const t = createMaterialTable();
    expect(t.materials.length).toBe(MATERIAL_PRESETS.length);
  });
  it("rejects hull 'refit' without deform 'joints'", () => {
    const bad = { ...MATERIAL_PRESETS[0], name: 'bad', hull: 'refit' as const };
    expect(() => createMaterialTable([...MATERIAL_PRESETS, bad])).toThrow(/refit/);
  });
  it('rejects plastic material without epsPlasticMax', () => {
    const bad = { ...MATERIAL_PRESETS[0], name: 'bad', sigmaY: 10, epsPlasticMax: 0 };
    expect(() => createMaterialTable([bad])).toThrow(/epsPlasticMax/);
  });
  it('derived interface: series stiffness and weaker strengths, cached, presets untouched', () => {
    const t = createMaterialTable();
    const a = materialId(t, 'concrete'), b = materialId(t, 'steel');
    const id = interfaceMaterial(t, a, b);
    const m = t.materials[id];
    expect(m.E).toBeCloseTo(1 / (1 / 30 + 1 / 200), 12);
    expect(m.sigmaT).toBe(3);
    expect(m.sigmaY).toBe(0); // brittle side governs
    expect(interfaceMaterial(t, b, a)).toBe(id);
    expect(MATERIAL_PRESETS.length).toBe(createMaterialTable().materials.length);
  });
});

describe('grid generator', () => {
  const t = createMaterialTable();
  const concrete = materialId(t, 'concrete');
  const model = generateFromBoxes(gridBoxes({ counts: [3, 3, 3], cell: [1, 0.5, 2], material: concrete, supportBottom: true }), t);

  it('bonds = face-adjacent pairs, normal from A to B, frames orthonormal', () => {
    expect(model.bondCount).toBe(3 * 2 * 3 * 3);
    for (let b = 0; b < model.bondCount; b++) {
      const a = model.bondA[b], c = model.bondB[b];
      const d = [0, 1, 2].map((k) => model.chunkPos[3 * c + k] - model.chunkPos[3 * a + k]);
      const n = [0, 1, 2].map((k) => model.bondNormal[3 * b + k]);
      expect(d[0] * n[0] + d[1] * n[1] + d[2] * n[2]).toBeGreaterThan(0);
      const t1 = [0, 1, 2].map((k) => model.bondT1[3 * b + k]), t2 = [0, 1, 2].map((k) => model.bondT2[3 * b + k]);
      // n x t1 = t2 (right-handed frame R = [n t1 t2])
      expect(n[1] * t1[2] - n[2] * t1[1]).toBe(t2[0]);
      expect(n[2] * t1[0] - n[0] * t1[2]).toBe(t2[1]);
      expect(n[0] * t1[1] - n[1] * t1[0]).toBe(t2[2]);
      expect(model.bondCornerOffset[b + 1] - model.bondCornerOffset[b]).toBe(4);
    }
  });

  it('face moments for a rectangle', () => {
    // A bond along y (normal y): t1 = z (w1 = 2), t2 = x (w2 = 1). I1 = w1 w2^3/12, I2 = w2 w1^3/12.
    const b = [...Array(model.bondCount).keys()].find((i) => model.bondNormal[3 * i + 1] === 1)!;
    expect(model.bondArea[b]).toBeCloseTo(2, 12);
    expect(model.bondI1[b]).toBeCloseTo((2 * 1) / 12, 12);
    expect(model.bondI2[b]).toBeCloseTo((1 * 8) / 12, 12);
  });

  it('hull volume equals cell volume (closed, outward)', () => {
    for (let i = 0; i < model.chunkCount; i++) expect(hullVolume(model, i)).toBeCloseTo(model.chunkVolume[i], 12);
  });

  it('face tags: exactly one interior cell, outer area = block surface, inner area = 2 x bond areas', () => {
    expect(model.chunkCount - surfaceChunkCount(model)).toBe(1);
    const centre = [...Array(model.chunkCount).keys()].find((i) => !(model.chunkFlags[i] & ChunkFlag.SURFACE))!;
    expect([...model.chunkPos.subarray(3 * centre, 3 * centre + 3)]).toEqual([1.5, 0.75, 3]);
    const { outer, inner } = renderAreas(model);
    expect(outer).toBeCloseTo(2 * (3 * 1.5 + 1.5 * 6 + 3 * 6), 9);
    const bondArea = model.bondArea.reduce((s, x) => s + x, 0);
    expect(inner).toBeCloseTo(2 * bondArea, 9);
  });

  it('supports = bottom layer', () => {
    for (let i = 0; i < model.chunkCount; i++)
      expect(!!(model.chunkFlags[i] & ChunkFlag.SUPPORT)).toBe(model.chunkPos[3 * i + 1] < 0.5);
  });
});

describe('brick wall generator (partial face overlap)', () => {
  const t = createMaterialTable();
  const brick = materialId(t, 'brick'), mortar = materialId(t, 'mortar');
  const boxes = brickWallBoxes({ bricksPerRow: 4, rows: 4, brick: [0.2, 0.06, 0.1], material: brick, supportBottom: true });
  const model = generateFromBoxes(boxes, t, { interfaceFor: () => mortar });

  it('volumes sum to the wall; all bricks are surface cells', () => {
    const v = model.chunkVolume.reduce((s, x) => s + x, 0);
    expect(v).toBeCloseTo(0.8 * 0.24 * 0.1, 12);
    expect(surfaceChunkCount(model)).toBe(model.chunkCount);
  });

  it('course-to-course bonds split faces; inner area = 2 x bond area; outer = wall surface', () => {
    const { outer, inner } = renderAreas(model);
    expect(inner).toBeCloseTo(2 * model.bondArea.reduce((s, x) => s + x, 0), 12);
    expect(outer).toBeCloseTo(2 * (0.8 * 0.24 + 0.24 * 0.1 + 0.8 * 0.1), 12);
    // a full brick on top of a running-bond course touches two bricks below
    const vertical = [...Array(model.bondCount).keys()].filter((b) => model.bondNormal[3 * b + 1] === 1);
    expect(vertical.length).toBeGreaterThan(4 * 3);
    expect([...model.bondMaterial].every((m) => m === mortar)).toBe(true);
  });
});
