// Universal collision fracture (CollisionBreaker): any contact pair with enough approach energy cuts the touching chunks
// and keeps moving instead of bouncing; freed pieces collide normally (never fall through the ground).
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });

const table = createMaterialTable(), h = 0.4, concrete = materialId(table, 'concrete');
/** 5 x 5 x 1 concrete wall (z in [0, 0.4]), bottom row supported; centre chunk 12 at (1, 1, 0.2). */
const wall = () => generateFromBoxes(gridBoxes({ counts: [5, 5, 1], cell: [h, h, h], material: concrete, supportBottom: true }), table);

/** Fires a 0.15 m steel ball at chunk 12 along +z at speed v (no gravity); returns final ball z, vz and the sim. */
function shoot(v: number, energyScale = 1) {
  const model = wall(), physics = new RapierBackend({ gravity: [0, 0, 0], impactThreshold: 100 });
  const sim = new DestructionSim(model, table, physics, { collisionBreak: { energyScale } });
  const p = model.chunkPos;
  const ball = physics.createSphere(0.15, [p[36], p[37], -1], [0, 0, v], 7850, 0xffffffff);
  const st = new Float64Array(13);
  for (let k = 0; k < 240; k++) { physics.step(1 / 240); sim.tick(1 / 240); }
  physics.readBodyStates([ball], st);
  physics.dispose();
  return { vz: st[9], z: st[2], sim };
}

/** Beam of 5 concrete cubes at y = 2..2.4 hanging off support chunk 0; cutting bond 0-1 drops 4 chunks onto ground y = 0. */
function dropBeam(energyScale: number) {
  const boxes: Box[] = [];
  for (let x = 0; x < 5; x++) boxes.push({ min: [x * h, 2, 0], max: [(x + 1) * h, 2 + h, h], material: concrete, support: x === 0 });
  const model = generateFromBoxes(boxes, table), physics = new RapierBackend({ impactThreshold: 100 });
  physics.createGround(0, packGroups(0xffff, 0xffff));
  const sim = new DestructionSim(model, table, physics, { collisionBreak: { energyScale } });
  const b01 = Array.from({ length: model.bondCount }, (_, b) => b).filter((b) => model.bondA[b] + model.bondB[b] === 1);
  sim.removeBonds(b01);
  const pose = new Float32Array(7);
  let minY = Infinity;
  for (let k = 0; k < 480; k++) { physics.step(1 / 240); sim.tick(1 / 240); }
  for (const isl of sim.handoff.islands) {
    if (!isl) continue;
    physics.readBodyPoses([isl.body], pose);
    minY = Math.min(minY, pose[1]);
  }
  physics.dispose();
  return { sim, minY };
}

describe('collision breaking', () => {
  it('a fast ball cuts the hit chunk free and keeps moving forward', () => {
    const { vz, z, sim } = shoot(20);
    expect(sim.isStatic[12]).toBe(0);
    expect(sim.breaker.broken).toBeGreaterThan(0);
    expect(vz).toBeGreaterThan(1); // not bounced
    expect(z).toBeGreaterThan(h);
  });

  it('fracture energy is paid by the pair: a costlier wall slows the ball more', () => {
    const cheap = shoot(20).vz, dear = shoot(20, 30).vz; // E_break(12) ~ 336 J x 30 ~ 10 kJ of the ~22 kJ ball
    expect(dear).toBeLessThan(cheap); // both also shove the freed chunk, so only the order is exact
  });

  it('an unbreakable wall bounces the ball and stays intact', () => {
    const { vz, sim } = shoot(20, Infinity);
    expect(sim.brokenTotal).toBe(0);
    expect(vz).toBeLessThanOrEqual(0);
  });

  it('a falling beam breaks on the ground, and its pieces stay above the ground', () => {
    const tough = dropBeam(Infinity), weak = dropBeam(1);
    expect(tough.sim.breaker.broken).toBe(0);
    expect(weak.sim.breaker.broken).toBeGreaterThan(0);
    expect(weak.sim.brokenTotal).toBeGreaterThan(tough.sim.brokenTotal);
    expect(weak.minY).toBeGreaterThan(0); // centres above the ground: no pass-through leak
    expect(tough.minY).toBeGreaterThan(0);
  });
});
