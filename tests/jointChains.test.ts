// Joint chains (NOTES Design §4b, M4 tests): stability, permanent set after unload, fracture removes the joint and separates bodies.
import { beforeAll, describe, expect, it } from 'vitest';
import { Material, MATERIAL_PRESETS, createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { DestructionSim } from '../src/sim/DestructionSim';
import { PlasticViews, updateJointPlasticity } from '../src/kernels/jointPlasticity';

beforeAll(async () => { await initRapier(); });

const steel = MATERIAL_PRESETS.find((m) => m.name === 'steel')!;
/** Test records (data-driven: new behaviour = new record). Root stress under own weight ~1.9 MPa for the chain below. */
const extra: Material[] = [
  { ...steel, name: 'softSteel', sigmaY: 0.5, H: 0.02 }, // yields under own weight, hardens before breaking
  { ...steel, name: 'softElastic', sigmaY: 0, epsElasticMax: 1 }, // same stiffness, never yields or breaks
  { ...steel, name: 'holdSteel', sigmaY: 2.5, H: 0.02 }, // holds its own weight, yields under a push (a sustained load above yield ratchets via bakes, NOTES)
  { ...steel, name: 'brittleSteel', sigmaY: 0.5, H: 0.02, epsPlasticMax: 0.01 }, // yields then breaks
];
const table = createMaterialTable([...MATERIAL_PRESETS, ...extra]);

/** Cantilever of n cubes (h = 0.1) along x, chunk 0 = support; promotes chunk 1's part; runs `seconds` at 60 Hz x 4 substeps. */
function cantilever(material: string, n = 10, demoteTime = Infinity) {
  const h = 0.1, mat = materialId(table, material), boxes: Box[] = [];
  for (let k = 0; k < n; k++) boxes.push({ min: [k * h, 1, 0], max: [(k + 1) * h, 1 + h, h], material: mat, support: k === 0 });
  const model = generateFromBoxes(boxes, table);
  const physics = new RapierBackend({ solverIterations: 16 });
  const sim = new DestructionSim(model, table, physics, { chains: { promoteForce: Infinity, demoteTime } });
  expect(sim.chains.promote(1)).toBe(n - 1);
  const tipBody = sim.chains.bodies[sim.chains.chunkBody[n - 1]];
  const pose = new Float32Array(7), y0 = model.chunkPos[3 * (n - 1) + 1];
  const run = (seconds: number) => {
    for (let s = 0; s < Math.round(seconds * 60); s++) for (let k = 0; k < 4; k++) { physics.step(1 / 240); sim.tick(1 / 240); }
    physics.readBodyPoses([tipBody], pose);
    return y0 - pose[1]; // tip sag (m)
  };
  return { model, physics, sim, run };
}

/** Kicks the outer half of a cantilever downward (transient load that bends it plastically). */
function pushTip(c: ReturnType<typeof cantilever>, n: number): void {
  for (let k = n >> 1; k < n; k++) c.physics.world.getRigidBody(c.sim.chains.bodies[c.sim.chains.chunkBody[k]]).setLinvel({ x: 0, y: -1.5 * k / n, z: 0 }, true);
}

describe('joint plasticity kernel', () => {
  it('below yield nothing moves; above yield the target returns the moment to the yield moment', () => {
    const v: PlasticViews = {
      cornerOffset: Uint32Array.of(0, 4), cornerYZ: Float64Array.of(-0.05, -0.05, 0.05, -0.05, 0.05, 0.05, -0.05, 0.05),
      I1: Float64Array.of(0.1 ** 4 / 12), I2: Float64Array.of(0.1 ** 4 / 12), J: Float64Array.of(0.1406 * 0.1 ** 4),
      length: Float64Array.of(0.1), rMax: Float64Array.of(Math.SQRT2 * 0.05), k: Float64Array.of(1e5, 1e5, 1e5),
      sigmaY: Float64Array.of(1e6), H: Float64Array.of(0), E: Float64Array.of(2e11), sigmaT: Float64Array.of(4e8),
      epsPlasticMax: Float64Array.of(1), epsElasticMax: Float64Array.of(0),
      target: new Float64Array(3), epsP: new Float64Array(1), alive: Uint8Array.of(1),
    };
    const ch = new Uint8Array(1), br = new Uint8Array(1), ratio = new Float64Array(1);
    // yield moment about t1: M_y = sigmaY I1 / z_max = 1e6 * 8.33e-6 / 0.05 = 166.7 N m -> phi_y = 1.667e-3
    updateJointPlasticity(v, Float64Array.of(0, 1e-3, 0), ch, br, ratio, 1);
    expect(ch[0]).toBe(0);
    expect(ratio[0]).toBeCloseTo(0.6, 6);
    updateJointPlasticity(v, Float64Array.of(0, 4e-3, 0), ch, br, ratio, 1);
    expect(ch[0]).toBe(1);
    expect(4e-3 - v.target[1]).toBeCloseTo(1e-3 / 0.6, 9); // elastic part back at phi_y
    expect(v.epsP[0]).toBeGreaterThan(0);
  });
});

describe('joint chains (cantilever, 4 substeps, 16 iterations)', () => {
  it('steel: stable over 10 s, small sag, no drift, no breaks', () => {
    const c = cantilever('steel');
    const s5 = c.run(5), s10 = c.run(5);
    console.log(`[chains steel] sag=${s10.toExponential(2)}`);
    expect(Number.isFinite(s10)).toBe(true);
    expect(s10).toBeGreaterThan(0);
    expect(s10).toBeLessThan(0.03);
    expect(Math.abs(s10 - s5)).toBeLessThan(0.01 * s10 + 1e-5);
    expect(c.sim.chains.brokenJoints).toBe(0);
    c.physics.dispose();
  });

  it('steel: tip sag matches the discrete hinge-chain prediction with scaled stiffness (and records raw Euler-Bernoulli)', () => {
    // Hinge chain: joint j at face x_j = j h (j = 1..n-1), rotational stiffness k = s E I / h, I = h^4/12, s = stiffnessScale.
    // M_j = sum_{m >= j} m_c g ((m + 1/2) h - x_j); tip sag = sum_j (M_j / k) ((n - 1/2) h - x_j).
    // Continuum E-B (uniform load q over length Lb = (n-1) h, clamped): w = q Lb^4 / (8 E I).
    const n = 10, h = 0.1, g = 9.81, mc = steel.density * h ** 3, I = h ** 4 / 12, E = steel.E * 1e9, s = 1e-2;
    const k = s * E * I / h;
    let hinge = 0;
    for (let j = 1; j < n; j++) {
      let M = 0;
      for (let m = j; m < n; m++) M += mc * g * ((m + 0.5) * h - j * h);
      hinge += (M / k) * ((n - 0.5) * h - j * h);
    }
    const eb = (mc / h) * g * ((n - 1) * h) ** 4 / (8 * E * I);
    const c = cantilever('steel', n);
    const sag = c.run(5);
    console.log(`[chains EB] sim=${sag.toExponential(2)} hinge(s=1e-2)=${hinge.toExponential(2)} EB(raw E)=${eb.toExponential(2)} EB(s E)=${(eb / s).toExponential(2)}`);
    // Recorded 2026-11-15: sim/hinge = 1.36 (solver compliance on top of the motors, 16 iters x 4 substeps). Guard the band.
    expect(sag / hinge).toBeGreaterThan(1.0);
    expect(sag / hinge).toBeLessThan(1.6);
    c.physics.dispose();
  });

  it('permanent set after unload (plastic) vs spring-back (elastic)', () => {
    const out: Record<string, [number, number]> = {};
    for (const mat of ['softSteel', 'softElastic']) {
      const c = cantilever(mat);
      const loaded = c.run(4);
      c.physics.world.gravity = { x: 0, y: 0, z: 0 };
      for (const b of c.sim.chains.bodies) c.physics.world.getRigidBody(b).wakeUp(); // sleeping bodies ignore gravity changes
      const unloaded = c.run(4);
      out[mat] = [loaded, unloaded];
      expect(c.sim.chains.brokenJoints).toBe(0);
      c.physics.dispose();
    }
    console.log(`[chains set] plastic loaded/unloaded=${out.softSteel.map((x) => x.toExponential(2))} elastic=${out.softElastic.map((x) => x.toExponential(2))}`);
    expect(out.softSteel[0]).toBeGreaterThan(out.softElastic[0]);
    expect(out.softSteel[1]).toBeGreaterThan(0.02); // set stays
    expect(out.softSteel[1]).toBeGreaterThan(0.5 * out.softSteel[0]);
    expect(Math.abs(out.softElastic[1])).toBeLessThan(2e-3); // springs back
  });

  it('fracture removes the joint, breaks the bond in integrity and the chain falls', () => {
    const c = cantilever('brittleSteel');
    const sag = c.run(3);
    expect(c.sim.chains.brokenJoints).toBeGreaterThan(0);
    const broken = [...c.sim.chains.bondJoint.keys()].filter((b) => c.sim.chains.jointState(b)?.alive === false);
    expect(broken.length).toBeGreaterThan(0);
    for (const b of broken) expect(c.sim.alive[b]).toBe(0); // integrity saw the removal
    expect(sag).toBeGreaterThan(1); // the tip piece falls freely
    expect(c.sim.handoff.islands.length).toBe(9); // no extra island for promoted chunks
    c.physics.dispose();
  });

  it('demotion: a bent part at rest freezes into static colliders at its deformed pose; re-promotion and cuts do not pop', () => {
    const n = 10, c = cantilever('holdSteel', n, 0.5), { sim, model, physics } = c, h = sim.handoff;
    pushTip(c, n);
    const tipBody = sim.chains.bodies[sim.chains.chunkBody[n - 1]], last = new Float32Array(7);
    let steps = 0;
    while (!sim.chains.demoted && steps < 60 * 4 * 10) {
      physics.readBodyPoses([tipBody], last);
      physics.step(1 / 240); sim.tick(1 / 240); steps++;
    }
    expect(sim.chains.demoted).toBe(n - 1);
    expect(h.islands.every((i) => i.removed)).toBe(true);
    for (let k = 0; k < n; k++) expect(sim.isStatic[k]).toBe(1);
    expect(sim.alive.every((a) => a === 1)).toBe(true); // demotion keeps bonds
    const tip = 3 * (n - 1), sag = model.chunkPos[tip + 1] - h.setPos[tip + 1];
    console.log(`[chains demote] after ${(steps / 240).toFixed(2)} s, frozen tip sag=${sag.toExponential(2)}`);
    expect(sag).toBeGreaterThan(0.02); // plastic set kept
    for (let i = 0; i < 3; i++) expect(h.setPos[tip + i]).toBeCloseTo(last[i], 3); // frozen where it rested
    expect(h.staticPoseDirty.length).toBe(n - 1);

    // re-promotion starts from the frozen shape (rest-state bake), no spring back
    expect(sim.chains.promote(1)).toBe(n - 1);
    const frozen = h.setPos.slice(tip, tip + 3), now = new Float32Array(7);
    physics.step(1 / 240); sim.tick(1 / 240);
    physics.readBodyPoses([sim.chains.bodies[sim.chains.chunkBody[n - 1]]], now);
    for (let i = 0; i < 3; i++) expect(Math.abs(now[i] - frozen[i])).toBeLessThan(2e-3);
    for (let k = 0; k < 240; k++) { physics.step(1 / 240); sim.tick(1 / 240); }
    physics.readBodyPoses([sim.chains.bodies[sim.chains.chunkBody[n - 1]]], now);
    const sag2 = model.chunkPos[tip + 1] - now[1];
    console.log(`[chains demote] re-promoted 1 s later sag=${sag2.toExponential(2)}`);
    expect(Math.abs(sag2 - sag)).toBeLessThan(0.1 * sag); // resumes in equilibrium: no spring back, no new drop
    physics.dispose();
  });

  it('cutting a demoted bent part re-promotes it as a free joint chain at the frozen pose', () => {
    const n = 10, c = cantilever('holdSteel', n, 0.5), { sim, model, physics } = c, h = sim.handoff;
    pushTip(c, n);
    for (let s = 0; s < 60 * 4 * 10 && !sim.chains.demoted; s++) { physics.step(1 / 240); sim.tick(1 / 240); }
    expect(sim.chains.demoted).toBe(n - 1);
    const frozen = h.setPos.slice();
    let root = -1;
    for (let b = 0; b < model.bondCount; b++) if (model.bondA[b] + model.bondB[b] === 1) root = b;
    sim.removeBonds([root]);
    physics.step(1 / 240); sim.tick(1 / 240);
    // joint chunks cut loose while static stay bendable (no stiff rigid island); bodies start at the frozen pose
    const pose = new Float32Array(7);
    for (let k = 1; k < n; k++) {
      expect(sim.chains.chunkBody[k]).toBeGreaterThanOrEqual(0);
      physics.readBodyPoses([sim.chains.bodies[sim.chains.chunkBody[k]]], pose);
      for (let i = 0; i < 3; i++) expect(pose[i]).toBeCloseTo(frozen[3 * k + i], 2);
    }
    expect(sim.chains.aliveJoints).toBe(n - 2);
    physics.dispose();
  });
});
