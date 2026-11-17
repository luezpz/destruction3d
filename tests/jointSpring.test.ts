// M4 spike (NOTES Design §4b): explicit joint springs (option a) vs Rapier fixed joints (option b) on a cantilever chain.
import { beforeAll, describe, expect, it } from 'vitest';
import { BODY_STATE_STRIDE, JointViews, capJointStiffness, computeJointSprings } from '../src/kernels/jointSpring';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { chunkHull } from '../src/physics/fromModel';
import { GroupBit, packGroups } from '../src/physics/PhysicsBackend';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';

beforeAll(async () => { await initRapier(); });

/** Chain joints between consecutive bodies along +x (cube size h), bond frame = body axes [x y z]. */
function chainViews(m: number, h: number): JointViews {
  const v: JointViews = {
    jointA: new Uint32Array(m), jointB: new Uint32Array(m), anchorA: new Float64Array(3 * m), anchorB: new Float64Array(3 * m),
    frame: new Float64Array(9 * m), restQ: new Float64Array(4 * m), restD: new Float64Array(3 * m),
    k: new Float64Array(6 * m), c: new Float64Array(6 * m), alive: new Uint8Array(m).fill(1),
  };
  for (let j = 0; j < m; j++) {
    v.jointA[j] = j; v.jointB[j] = j + 1;
    v.anchorA[3 * j] = h / 2; v.anchorB[3 * j] = -h / 2;
    v.frame[9 * j] = 1; v.frame[9 * j + 4] = 1; v.frame[9 * j + 8] = 1;
    v.restQ[4 * j + 3] = 1;
  }
  return v;
}

/** Rest states for n bodies at x = i h, identity rotation, at rest. */
function restState(n: number, h: number): Float64Array {
  const s = new Float64Array(BODY_STATE_STRIDE * n);
  for (let i = 0; i < n; i++) { s[BODY_STATE_STRIDE * i] = i * h; s[BODY_STATE_STRIDE * i + 6] = 1; }
  return s;
}

describe('joint spring kernel', () => {
  const h = 0.1, v = chainViews(1, h);
  v.k.fill(1000);
  const err = new Float64Array(6), f = new Float64Array(6), w = new Float64Array(12);

  it('rest pose has zero error and wrench', () => {
    computeJointSprings(v, restState(2, h), err, f, w, 1);
    expect(Math.max(...err.map(Math.abs), ...w.map(Math.abs))).toBe(0);
  });

  it('stretch gives tension pulling bodies together; wrench balances (sum F = 0, sum M about origin = 0)', () => {
    const s = restState(2, h);
    s[BODY_STATE_STRIDE] += 0.01; // stretch
    s[BODY_STATE_STRIDE + 1] += 0.004; // shear
    const a = 0.02; // twist + bend of b about z
    s[BODY_STATE_STRIDE + 5] = Math.sin(a / 2); s[BODY_STATE_STRIDE + 6] = Math.cos(a / 2);
    w.fill(0);
    computeJointSprings(v, s, err, f, w, 1);
    expect(err[0]).toBeCloseTo(0.01 + (h / 2) * (1 - Math.cos(a)), 9);
    expect(err[5]).toBeCloseTo(a, 6);
    expect(f[0]).toBeLessThan(0); // pulls b back (-x)
    for (let i = 0; i < 3; i++) expect(w[i] + w[6 + i]).toBeCloseTo(0, 9);
    // moment about origin: tau + x cross F per body
    const xb = [s[BODY_STATE_STRIDE], s[BODY_STATE_STRIDE + 1], 0];
    const Mz = w[5] + w[11] + (xb[0] * w[7] - xb[1] * w[6]);
    expect(Mz).toBeCloseTo(0, 9);
  });

  it('does not allocate per call', () => {
    const s = restState(2, h);
    const orig = Float64Array; let n = 0;
    (globalThis as { Float64Array: unknown }).Float64Array = new Proxy(orig, { construct(t, a) { n++; return Reflect.construct(t, a); } });
    try { for (let i = 0; i < 100; i++) computeJointSprings(v, s, err, f, w, 1); } finally { globalThis.Float64Array = orig; }
    expect(n).toBe(0);
  });
});

interface ChainResult { sag: number; predicted: number; eb: number; drift: number; finite: boolean; rootMoment: number; rootExact: number }

/**
 * Steel cantilever of n cubes (h) along x, body 0 fixed; runs `seconds` at 60 Hz x substeps.
 * mode 'springs' = explicit capped springs (omegaMax = 1/(2 dt_sub)), 'fixed' = Rapier fixed joints.
 * @returns tip sag (m, averaged over the last second), predicted sag for the capped springs, Euler-Bernoulli sag, drift = |sag(last s) - sag(previous s)|.
 */
function runChain(mode: 'springs' | 'fixed' | 'motors', n: number, substeps: number, seconds: number, gravity: number, omegaFactor = 0.5, motorScale = 1, solverIterations = 4): ChainResult {
  const h = 0.1, t = createMaterialTable(), steel = materialId(t, 'steel');
  const model = generateFromBoxes(gridBoxes({ counts: [n, 1, 1], cell: [h, h, h], material: steel }), t);
  const be = new RapierBackend({ gravity: [0, -gravity, 0], solverIterations });
  const groups = packGroups(GroupBit.GRACE, 0); // no contacts inside the chain (spike only)
  const mass = 7850 * h ** 3, I = mass * h * h / 6;
  const bodies: number[] = [];
  const support = be.createStaticBody();
  be.createStaticChunk(support, chunkHull(model, 0), groups);
  bodies.push(support);
  const x0 = model.chunkPos[0];
  for (let i = 1; i < n; i++) {
    const p: [number, number, number] = [model.chunkPos[3 * i], model.chunkPos[3 * i + 1], model.chunkPos[3 * i + 2]];
    bodies.push(be.createDynamicIsland({
      position: p, rotation: [0, 0, 0, 1], chunks: [chunkHull(model, i, p)], mass, inertia: [I, I, I], inertiaFrame: [0, 0, 0, 1],
      linvel: [0, 0, 0], angvel: [0, 0, 0], groups,
    }).body);
  }
  const dt = 1 / 60 / substeps, m = n - 1, v = chainViews(m, h);
  // the support body sits at the world origin: joint 0's anchor on it is chunk 0's face centre
  const a0: [number, number, number] = [x0 + h / 2, model.chunkPos[1], model.chunkPos[2]];
  v.anchorA.set(a0, 0);
  // raw bond stiffness (NOTES Design §2): A = h^2, I = h^4/12, J ~ 0.1406 h^4, L = h
  const E = 200e9, G = E / 2.6, A = h * h, Ib = h ** 4 / 12, J = 0.1406 * h ** 4;
  const kRaw = new Float64Array(6 * m);
  for (let j = 0; j < m; j++) kRaw.set([E * A / h, (5 / 6) * G * A / h, (5 / 6) * G * A / h, G * J / h, E * Ib / h, E * Ib / h], 6 * j);
  // reduced mass/inertia; joint 0 is against the fixed support, so the full body
  const mEff = new Float64Array(m).map((_, j) => (j === 0 ? mass : mass / 2));
  const iEff = new Float64Array(m).map((_, j) => (j === 0 ? I : I / 2));
  capJointStiffness(kRaw, mEff, iEff, new Float64Array(m).fill(h / 2), omegaFactor / dt, 0.5, v.k, v.c, m);
  if (mode === 'motors') {
    // option c in Rapier: spherical joint (translation locked) + implicit angular motors at the raw bond stiffness
    const id: [number, number, number, number] = [0, 0, 0, 1];
    for (let j = 0; j < m; j++) {
      const k: [number, number, number] = [kRaw[6 * j + 3] * motorScale, kRaw[6 * j + 4] * motorScale, kRaw[6 * j + 5] * motorScale];
      const c = k.map((kk) => Math.sqrt(kk * I)) as [number, number, number]; // zeta 0.5
      be.createSpringJoint(bodies[j], bodies[j + 1], j === 0 ? a0 : [h / 2, 0, 0], [-h / 2, 0, 0], id, id, k, c);
      for (let i = 3; i < 6; i++) v.k[6 * j + i] = kRaw[6 * j + i] * motorScale; // kernel only reads errors -> moment readout
    }
  }
  if (mode === 'fixed') for (let j = 0; j < m; j++) be.createJoint('fixed', bodies[j], bodies[j + 1], j === 0 ? a0 : [h / 2, 0, 0], [-h / 2, 0, 0]);

  const state = new Float64Array(BODY_STATE_STRIDE * n), err = new Float64Array(6 * m), f = new Float64Array(6 * m), wr = new Float64Array(6 * n);
  const pose = new Float32Array(7);
  const tip = [bodies[n - 1]];
  let sum = 0, cnt = 0, prevAvg = 0, finite = true;
  const steps = Math.round(seconds * 60);
  for (let s = 0; s < steps; s++) {
    for (let k = 0; k < substeps; k++) {
      if (mode === 'springs') {
        be.readBodyStates(bodies, state);
        wr.fill(0);
        computeJointSprings(v, state, err, f, wr, m);
        be.applyWrenches(bodies, wr, dt);
      }
      be.step(dt);
    }
    be.readBodyPoses(tip, pose);
    if (!Number.isFinite(pose[1])) finite = false;
    if (s >= steps - 120) { sum += -pose[1]; cnt++; }
    if (s === steps - 61) { prevAvg = sum / cnt; sum = 0; cnt = 0; }
  }
  // root bending moment readout M_0 = k_z phi_z (joint 0) vs statics sum m g (x_i - x_0)
  be.readBodyStates(bodies, state);
  wr.fill(0);
  computeJointSprings(v, state, err, f, wr, m);
  const rootMoment = Math.abs(v.k[5] * err[5]);
  let rootExact = 0;
  for (let i = 1; i < n; i++) rootExact += mass * gravity * (model.chunkPos[3 * i] - (x0 + h / 2));
  be.dispose();
  const sag = sum / cnt - -model.chunkPos[1];
  const prevSag = prevAvg - -model.chunkPos[1];
  // linear statics of the capped chain: theta_j = M_j / k_bend, slip_j = V_j / k_shear, tip = sum theta_j (x_tip - x_j) + slip_j
  const xTip = model.chunkPos[3 * (n - 1)];
  let predicted = 0;
  for (let j = 0; j < m; j++) {
    const xj = x0 + (j + 0.5) * h;
    let V = 0, M = 0;
    for (let i = j + 1; i < n; i++) { V += mass * gravity; M += mass * gravity * (model.chunkPos[3 * i] - xj); }
    predicted += (M / v.k[6 * j + 5]) * (xTip - xj) + V / v.k[6 * j + 1];
  }
  const Lb = (n - 1) * h, q = mass * gravity / h;
  const eb = q * Lb ** 4 / (8 * E * Ib);
  return { sag, predicted, eb, drift: Math.abs(sag - prevSag), finite, rootMoment, rootExact };
}

describe('M4 spike: cantilever chain 10 s', () => {
  it('(a) explicit springs: kernel matches the capped linear chain (short chain, small load)', () => {
    const r = runChain('springs', 4, 4, 10, 0.05);
    expect(r.finite).toBe(true);
    expect(Math.abs(r.sag - r.predicted) / r.predicted).toBeLessThan(0.05);
    expect(r.drift).toBeLessThan(0.02 * r.sag);
  });

  it('(c) spherical joints + implicit angular motors: stable, small sag, root moment readout within 15%', () => {
    // chosen setup: 4 substeps, 16 solver iterations, motor stiffness = raw EI/L * 1e-2 (motor compliance dominates solver error)
    const r = runChain('motors', 10, 4, 10, 9.81, 0.5, 1e-2, 16);
    console.log(`[spike motors] sag=${r.sag.toExponential(2)} EB=${r.eb.toExponential(2)} drift=${r.drift.toExponential(1)} M0/exact=${(r.rootMoment / r.rootExact).toFixed(3)}`);
    expect(r.finite).toBe(true);
    expect(r.sag).toBeLessThan(0.02);
    expect(r.drift).toBeLessThan(0.01 * r.sag + 1e-6);
    expect(Math.abs(r.rootMoment / r.rootExact - 1)).toBeLessThan(0.15);
  });

  it('records full-gravity sag: (a) springs vs (b) fixed joints (logged, NOTES.md)', () => {
    for (const sub of [1, 4]) {
      const s = runChain('springs', 10, sub, 10, 9.81), f = runChain('fixed', 10, sub, 10, 9.81);
      console.log(`[spike g=9.81 substeps=${sub}] springs sag=${s.sag.toExponential(3)} (pred ${s.predicted.toExponential(3)}) fixed sag=${f.sag.toExponential(3)} EB=${s.eb.toExponential(3)}`);
      expect(f.finite).toBe(true);
    }
  });
});
