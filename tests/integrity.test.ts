import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { BondSolver } from '../src/integrity/BondSolver';
import { StressTier } from '../src/integrity/StressTier';
import { chunkMasses, gravityLoads } from '../src/integrity/loads';

const table = createMaterialTable();
const concrete = materialId(table, 'concrete');
const glass = materialId(table, 'glass');

/** Horizontal chain of N+1 cubes (side h) along x, chunk 0 support. */
function cantilever(N: number, h: number, material: number) {
  const boxes: Box[] = [];
  for (let k = 0; k <= N; k++) boxes.push({ min: [k * h, 0, 0], max: [(k + 1) * h, h, h], material, support: k === 0 });
  return generateFromBoxes(boxes, table);
}

/** Solve to convergence then run one tier-1 step. */
function solveStep(s: BondSolver, t: StressTier, f: Float64Array, dt: number): number {
  s.begin(f);
  expect(s.iterate(10000, 1e-10)).toBe(true);
  return t.step(dt);
}

describe('tier 1 failure ratio', () => {
  it('cantilever root ratio = tension from M c / I', () => {
    const N = 6, h = 0.2, P = 200;
    const m = cantilever(N, h, concrete), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = -P;
    solveStep(s, t, f, 0);
    const M = P * (N - 0.5) * h, sigma = (M * (h / 2)) / (h ** 4 / 12);
    expect(t.ratio[0]).toBeCloseTo(sigma / 3e6, 6); // concrete sigmaT = 3 MPa governs (bending: tension = compression, sigmaC 10x)
    // ratios fall towards the tip
    for (let b = 1; b < m.bondCount; b++) expect(t.ratio[b]).toBeLessThan(t.ratio[b - 1]);
  });

  it('pure tension bar: ratio = N / (A sigmaT)', () => {
    const N = 3, h = 0.1, F = 1e4;
    const m = cantilever(N, h, concrete), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N] = F;
    solveStep(s, t, f, 0);
    for (let b = 0; b < m.bondCount; b++) expect(t.ratio[b]).toBeCloseTo(F / (h * h) / 3e6, 6);
  });
});

describe('tier 1 damage', () => {
  it('overloaded cantilever: no break on first step (low-pass), cracks after a delay, tip falls', () => {
    const N = 6, h = 0.2;
    const m = cantilever(N, h, concrete), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(chunkMasses(m, table), 9.81, f);
    f[6 * N + 1] -= 8000; // root ratio ~2.3
    expect(solveStep(s, t, f, 1 / 60)).toBe(0);
    let time = 1 / 60, broke = 0;
    while (!broke && time < 5) { broke = solveStep(s, t, f, 1 / 60); time += 1 / 60; }
    expect(broke).toBeGreaterThan(0);
    expect(time).toBeGreaterThan(0.05); // delay from filter + damage growth
    expect(t.broken[0]).toBe(0); // root bond goes first
    expect(t.detachedCount).toBe(1);
    expect(t.detachedOffset[1]).toBe(N); // every non-support chunk falls
    for (let i = 1; i <= N; i++) expect(s.active[i]).toBe(0);
    // static solve keeps working after the detach
    s.begin(f);
    expect(s.iterate(1000, 1e-8)).toBe(true);
  });

  it('a single short spike does not break a bond (filter)', () => {
    const N = 3, h = 0.1;
    const m = cantilever(N, h, concrete), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N] = 5 * 3e6 * h * h; // ratio 5
    expect(solveStep(s, t, f, 0.002)).toBe(0);
    f.fill(0);
    for (let k = 0; k < 30; k++) expect(solveStep(s, t, f, 1 / 60)).toBe(0);
  });

  it('glass: instant failure once filtered ratio > 1; propagation chains into pre-damaged neighbours', () => {
    const N = 4, h = 0.1;
    const m = cantilever(N, h, glass), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    s.damage[1] = 0.5; // 0.5 + 0.6 propagation >= 1 -> breaks in the same step
    s.updateStiffness();
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = -(40e6 * h ** 3 / 6) / ((N - 0.5) * h) * 1.2; // root ratio 1.2, bond 1 ~0.86 (breaks only via propagation)
    let broke = 0;
    for (let k = 0; k < 20 && !broke; k++) broke = solveStep(s, t, f, 1 / 60);
    expect(broke).toBe(2);
    expect(Array.from(t.broken.subarray(0, 2))).toEqual([0, 1]);
    expect(s.damage[2]).toBeCloseTo(0.6, 12); // got propagation from bond 1, not enough to break
    expect(t.detachedCount).toBe(2); // {1} and {2..N}
  });
});

describe('tier 0 connectivity', () => {
  it('removing the wall under a slab drops it; removing a redundant bond does not', () => {
    // two columns (x=0 and x=2) of height 2 under a 3-wide slab at y=2
    const boxes: Box[] = [];
    for (const x of [0, 2]) for (let y = 0; y < 2; y++) boxes.push({ min: [x, y, 0], max: [x + 1, y + 1, 1], material: concrete, support: y === 0 });
    for (let x = 0; x < 3; x++) boxes.push({ min: [x, 2, 0], max: [x + 1, 3, 1], material: concrete, support: false });
    const m = generateFromBoxes(boxes, table);
    const s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const bondBetween = (p: number, q: number) => {
      for (let b = 0; b < m.bondCount; b++) if ((m.bondA[b] === p && m.bondB[b] === q) || (m.bondA[b] === q && m.bondB[b] === p)) return b;
      throw new Error('no bond');
    };
    // cut the left column (chunk 1 -> slab chunk 4): slab still held by the right column
    expect(t.removeBonds([bondBetween(1, 4)])).toBe(0);
    // cut the right column too: the 3 slab chunks fall, upper column chunks stay on their bases
    expect(t.removeBonds([bondBetween(3, 6)])).toBe(1);
    const comp = Array.from(t.detachedChunks.subarray(0, t.detachedOffset[1])).sort();
    expect(comp).toEqual([4, 5, 6]);
  });

  it('splitting a hanging piece yields separate components and nothing grounded', () => {
    const m = cantilever(4, 0.1, concrete), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    expect(t.removeBonds([0, 2])).toBe(2); // chunks {1,2} and {3,4}
    const sizes = [t.detachedOffset[1] - t.detachedOffset[0], t.detachedOffset[2] - t.detachedOffset[1]];
    expect(sizes).toEqual([2, 2]);
  });
});
