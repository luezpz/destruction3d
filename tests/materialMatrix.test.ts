// Material matrix (PLAN §2b): every preset in three scenarios — cantilever under tip load, compressed column,
// impacted wall. Loads are scaled to a target root ratio from a unit-load probe; checks: no NaN, expected failure.
import { describe, expect, it } from 'vitest';
import { MATERIAL_PRESETS, createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { ChunkModel } from '../src/model/ChunkModel';
import { BondSolver } from '../src/integrity/BondSolver';
import { StressTier } from '../src/integrity/StressTier';

const table = createMaterialTable();
const h = 0.2, dt = 1 / 60, TARGET = 2;

/** Horizontal (axis 0) or vertical (axis 1) chain of N+1 cubes, chunk 0 a support. */
function chain(N: number, axis: 0 | 1, material: number): ChunkModel {
  const boxes: Box[] = [];
  for (let k = 0; k <= N; k++) {
    const min: [number, number, number] = [0, 0, 0], max: [number, number, number] = [h, h, h];
    min[axis] = k * h; max[axis] = (k + 1) * h;
    boxes.push({ min, max, material, support: k === 0 });
  }
  return generateFromBoxes(boxes, table);
}

/** Solves f (6n,) to convergence and returns the tier with fresh ratios (dt = 0: no damage). */
function probe(m: ChunkModel, f: Float64Array) {
  const s = new BondSolver(m, table), t = new StressTier(m, table, s);
  s.begin(f);
  expect(s.iterate(20000, 1e-10)).toBe(true);
  t.step(0);
  return { s, t };
}

/** Runs tier-1 steps under f until something breaks (max 10 s). @returns the tier after the first break. */
function runToBreak(m: ChunkModel, f: Float64Array) {
  const { s, t } = probe(m, f);
  for (let time = 0; time < 10; time += dt) {
    s.begin(f);
    s.iterate(20000, 1e-10);
    if (t.step(dt) > 0) break;
    for (let b = 0; b < m.bondCount; b++) expect(Number.isFinite(t.ratio[b]) && Number.isFinite(s.damage[b])).toBe(true);
  }
  return { s, t };
}

/** Index of the max finite ratio. */
function argmax(a: Float64Array): number {
  let k = 0;
  for (let i = 1; i < a.length; i++) if (a[i] > a[k]) k = i;
  return k;
}

describe.each(MATERIAL_PRESETS.map((p) => p.name))('material %s', (name) => {
  const id = materialId(table, name);

  it('cantilever: root bond fails first, tip falls', () => {
    const N = 5, m = chain(N, 0, id), f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = -1;
    const r1 = probe(m, f).t.ratio[0];
    expect(r1).toBeGreaterThan(0);
    f[6 * N + 1] = -TARGET / r1;
    const { t } = runToBreak(m, f);
    expect(t.brokenCount).toBeGreaterThan(0);
    expect(t.broken[0]).toBe(0);
    expect(t.detachedCount).toBe(1);
    expect(t.detachedOffset[1]).toBe(N);
  });

  it('compressed column: finite ratios, crushes (compression or elastic/plastic limit governs)', () => {
    const N = 4, m = chain(N, 1, id), f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = -1;
    const r1 = probe(m, f).t.ratio[0];
    f[6 * N + 1] = -TARGET / r1;
    const { t } = runToBreak(m, f);
    expect(t.brokenCount).toBeGreaterThan(0);
    // uniform axial force: every bond carries the same ratio (no shear/bending)
    for (let b = 1; b < m.bondCount; b++) expect(t.ratio[b]).toBeCloseTo(t.ratio[0], 6);
  });

  // Indeterminate structure: damaging bonds shed load to neighbours, so the first break is *a* highly loaded bond,
  // not necessarily the probe's argmax; it may be at the hit (local) or at the base (wall acts as a cantilever).
  it('impacted wall: a highly loaded bond fails, no NaN', () => {
    const nx = 4, ny = 4;
    const m = generateFromBoxes(gridBoxes({ counts: [nx, ny, 1], cell: [h, h, h], material: id, supportBottom: true }), table);
    const hit = (ny - 1) * nx + 1; // top row, second column
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * hit + 2] = 1; // out-of-plane push
    const pr = probe(m, f).t;
    const worst = argmax(pr.ratio);
    const probeRatio = pr.ratio.slice();
    f[6 * hit + 2] = (2 * TARGET) / probeRatio[worst];
    const { t } = runToBreak(m, f);
    expect(t.brokenCount).toBeGreaterThan(0);
    const firstProbe = Math.max(...Array.from(t.broken.subarray(0, t.brokenCount), (b) => probeRatio[b]));
    expect(firstProbe).toBeGreaterThan(0.5 * probeRatio[worst]);
    expect(t.detachedOffset[t.detachedCount]).toBeLessThanOrEqual(m.chunkCount - nx); // supports never detach
  });
});
