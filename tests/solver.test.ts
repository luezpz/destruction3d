import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { BondSolver } from '../src/integrity/BondSolver';
import { chunkMasses, chunkRotInertia, gravityLoads } from '../src/integrity/loads';
import { applyInertialRelief } from '../src/kernels/inertialRelief';
import { SHEAR_KAPPA } from '../src/kernels/bondStiffness';

const table = createMaterialTable();
const steel = materialId(table, 'steel');
const concrete = materialId(table, 'concrete');
const Esteel = 200e9, nuSteel = 0.3;

/** Straight chain of N+1 cubes-ish boxes (h along `axis`, b x b section); chunk 0 is a support if `support`. */
function chain(N: number, h: number, b: number, axis: 0 | 1, support: boolean) {
  const boxes: Box[] = [];
  for (let k = 0; k <= N; k++) {
    const min: [number, number, number] = [0, 0, 0], max: [number, number, number] = [b, b, b];
    min[axis] = k * h; max[axis] = (k + 1) * h;
    boxes.push({ min, max, material: steel, support: support && k === 0 });
  }
  return generateFromBoxes(boxes, table);
}

/** Solves to convergence from the current warm start. @returns iterations used. */
function solve(s: BondSolver, f: Float64Array, tol = 1e-10): number {
  s.begin(f);
  expect(s.iterate(10000, tol)).toBe(true);
  return s.iterations;
}

describe('bond solver', () => {
  it('axial bar: delta = F L / (E A)', () => {
    const N = 10, h = 0.5, b = 0.2, F = 1e5;
    const m = chain(N, h, b, 1, true), s = new BondSolver(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = F;
    solve(s, f);
    const u = new Float64Array(6 * m.chunkCount);
    s.displacement(u);
    expect(u[6 * N + 1]).toBeCloseTo((F * N * h) / (Esteel * b * b), 12);
    expect(Math.abs(u[6 * N + 1] / ((F * N * h) / (Esteel * b * b)) - 1)).toBeLessThan(1e-8);
  });

  it('cantilever tip deflection vs Euler-Bernoulli', () => {
    const N = 20, h = 0.5, b = 0.25, P = 1e3;
    const m = chain(N, h, b, 0, true), s = new BondSolver(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    f[6 * N + 1] = -P;
    solve(s, f);
    const u = new Float64Array(6 * m.chunkCount);
    s.displacement(u);
    const L = N * h, I = b ** 4 / 12, A = b * b, G = Esteel / (2 * (1 + nuSteel));
    const eb = (P * L ** 3) / (3 * Esteel * I);
    // discrete chain: midpoint rule of the bending integral + one shear spring per joint
    const discrete = eb - (P * L * h * h) / (12 * Esteel * I) + (P * L) / (SHEAR_KAPPA * G * A);
    const tip = -u[6 * N + 1];
    expect(Math.abs(tip / discrete - 1)).toBeLessThan(1e-6);
    expect(Math.abs(tip / eb - 1)).toBeLessThan(0.01);
    // root bond moment = P * lever (support face to tip centre)
    const F = new Float64Array(6 * m.bondCount), us = new Float64Array(6 * m.chunkCount);
    s.bondForces(F, us);
    const root = 0; // bond between chunk 0 and 1
    const M = Math.hypot(F[6 * root + 4], F[6 * root + 5]);
    expect(M).toBeCloseTo(P * (N - 0.5) * h, 3);
  });

  it('free island: self-equilibrated loads converge; inertial relief balances gravity + push', () => {
    const N = 8, h = 0.5, b = 0.2, F = 2e4;
    const m = chain(N, h, b, 0, false), s = new BondSolver(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    f[0] = -F; f[6 * N] = F; // pull ends apart
    solve(s, f);
    const bf = new Float64Array(6 * m.bondCount), us = new Float64Array(6 * m.chunkCount);
    s.bondForces(bf, us);
    for (let k = 0; k < m.bondCount; k++) expect(bf[6 * k] / F).toBeCloseTo(1, 6);

    const mass = chunkMasses(m, table), j = chunkRotInertia(m, mass);
    gravityLoads(mass, 9.81, f);
    f[6 * N + 2] += 5e3; // sideways push at one end -> needs rotation relief
    applyInertialRelief(f, mass, j, m.chunkPos, m.chunkCount);
    const sum = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < m.chunkCount; i++) {
      const p = m.chunkPos, o = 6 * i;
      for (let k = 0; k < 3; k++) sum[k] += f[o + k];
      sum[3] += p[3 * i + 1] * f[o + 2] - p[3 * i + 2] * f[o + 1] + f[o + 3];
      sum[4] += p[3 * i + 2] * f[o] - p[3 * i] * f[o + 2] + f[o + 4];
      sum[5] += p[3 * i] * f[o + 1] - p[3 * i + 1] * f[o] + f[o + 5];
    }
    for (const v of sum) expect(Math.abs(v)).toBeLessThan(1e-8);
    s.reset();
    solve(s, f, 1e-9);
  });

  // Warm start saves ~log(initial error ratio) iterations, so it pays at tier-1 tolerances (1e-4), not at 1e-8 (see NOTES).
  it('warm start after a damage step / break beats a cold solve at tier-1 tolerance', () => {
    const m = generateFromBoxes(gridBoxes({ counts: [10, 10, 2], cell: [0.5, 0.5, 0.5], material: concrete, supportBottom: true }), table);
    const s = new BondSolver(m, table), mass = chunkMasses(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(mass, 9.81, f);
    for (let b = 0; b < m.bondCount; b += 5) s.damage[b] = 0.3; // irregular (realistic) stiffness
    s.updateStiffness();
    solve(s, f, 1e-10);
    const tol = 1e-4;
    for (const step of [() => { s.damage[100] += 0.1; }, () => { s.alive[100] = 0; }]) {
      step();
      s.updateStiffness();
      const x0 = s.work.x.slice();
      const warm = solve(s, f, tol);
      s.reset();
      const cold = solve(s, f, tol);
      expect(warm).toBeLessThan(cold * 0.6);
      s.work.x.set(x0);
      solve(s, f, 1e-10);
    }
  });

  it('a break has a local effect', () => {
    const nx = 20, ny = 6;
    const m = generateFromBoxes(gridBoxes({ counts: [nx, ny, 1], cell: [0.5, 0.5, 0.5], material: concrete, supportBottom: true }), table);
    const s = new BondSolver(m, table), mass = chunkMasses(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(mass, 9.81, f);
    solve(s, f);
    const before = new Float64Array(6 * m.bondCount), after = new Float64Array(6 * m.bondCount), us = new Float64Array(6 * m.chunkCount);
    s.bondForces(before, us);
    // break a vertical bond near x = 0 (column 1, between rows 2 and 3)
    const cut = [...m.bondA.keys()].find((b) => {
      const c = m.bondCentroid;
      return Math.abs(m.bondNormal[3 * b + 1]) > 0.9 && Math.abs(c[3 * b] - 0.75) < 1e-6 && Math.abs(c[3 * b + 1] - 1.5) < 1e-6;
    })!;
    expect(cut).toBeDefined();
    s.alive[cut] = 0;
    s.updateStiffness();
    solve(s, f);
    s.bondForces(after, us);
    let near = 0, far = 0;
    for (let b = 0; b < m.bondCount; b++) {
      if (b === cut) continue;
      const d = Math.abs(after[6 * b] - before[6 * b]), x = m.bondCentroid[3 * b];
      if (x < 2) near = Math.max(near, d);
      if (x > 7) far = Math.max(far, d);
    }
    expect(near).toBeGreaterThan(0);
    expect(far).toBeLessThan(0.03 * near); // measured ~1%: elastic redistribution decays with distance
  });

  it('no typed-array allocation per stiffness update / solve / force readout', () => {
    const m = generateFromBoxes(gridBoxes({ counts: [6, 6, 2], cell: [0.5, 0.5, 0.5], material: concrete, supportBottom: true }), table);
    const s = new BondSolver(m, table), mass = chunkMasses(m, table);
    const f = new Float64Array(6 * m.chunkCount), bf = new Float64Array(6 * m.bondCount), us = new Float64Array(6 * m.chunkCount);
    gravityLoads(mass, 9.81, f);
    const g = globalThis as unknown as Record<string, unknown>;
    const names = ['Float64Array', 'Float32Array', 'Uint32Array', 'Int32Array', 'Uint8Array', 'Uint16Array'];
    const orig = names.map((n) => g[n]);
    let count = 0;
    names.forEach((n, k) => {
      g[n] = new Proxy(orig[k] as object, { construct(t, a, nt) { count++; return Reflect.construct(t as never, a, nt); } });
    });
    try {
      s.damage[0] = 0.5;
      s.updateStiffness();
      s.begin(f);
      s.iterate(50);
      s.bondForces(bf, us);
      applyInertialRelief(f, mass, mass, m.chunkPos, m.chunkCount);
    } finally {
      names.forEach((n, k) => { g[n] = orig[k]; });
    }
    expect(count).toBe(0);
  });

  it('benchmark: PCG iterations at ~1000 chunks', () => {
    const m = generateFromBoxes(gridBoxes({ counts: [10, 10, 10], cell: [0.5, 0.5, 0.5], material: concrete, supportBottom: true }), table);
    const s = new BondSolver(m, table), mass = chunkMasses(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(mass, 9.81, f);
    for (let b = 0; b < m.bondCount; b += 5) s.damage[b] = 0.3; // irregular stiffness (regular grids converge unrealistically fast)
    s.updateStiffness();
    const timed = (tol: number) => { const t = performance.now(); s.begin(f); expect(s.iterate(5000, tol)).toBe(true); return `${s.iterations} it ${(performance.now() - t).toFixed(1)} ms`; };
    const cold8 = timed(1e-8);
    s.reset();
    const cold4 = timed(1e-4);
    const bf = new Float64Array(6 * m.bondCount), us = new Float64Array(6 * m.chunkCount);
    s.bondForces(bf, us);
    let worst = 0;
    for (let b = 1; b < m.bondCount; b++) if (Math.abs(bf[6 * b]) > Math.abs(bf[6 * worst])) worst = b;
    s.alive[worst] = 0;
    const tu = performance.now();
    s.updateStiffness();
    const update = (performance.now() - tu).toFixed(1);
    const warm4 = timed(1e-4);
    console.log(`[bench] ${m.chunkCount} chunks, ${m.bondCount} bonds: cold 1e-8 ${cold8}; cold 1e-4 ${cold4}; updateStiffness ${update} ms; break most-loaded bond, warm 1e-4 ${warm4}`);
  });
});
