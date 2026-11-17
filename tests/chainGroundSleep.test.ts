// Regression (NOTES gotcha "zombie sleep"): curtain strips cut off the lintel land on the ground, our rest test sleeps them, a
// neighbour contact wakes part of a strip; the rest stayed flagged asleep in an active island and sank through the ground.
import { beforeAll, expect, it } from 'vitest';
import { MATERIAL_PRESETS, UNIT_SCALE, createMaterialTable, scaleMaterials } from '../src/model/Material';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';
import { InlineIntegrity } from '../src/integrity/IntegrityClient';
import { SCENE_STRENGTH, buildScene } from '../src/scenes';

beforeAll(async () => { await initRapier(); });

const ALL = packGroups(0xffff, 0xffff), h = 1 / 240;

it('dropped curtain strips rest on the ground and sleep, never sink', () => {
  const table = createMaterialTable(scaleMaterials(MATERIAL_PRESETS, UNIT_SCALE, SCENE_STRENGTH['rubber curtain'] ?? {}));
  const model = buildScene('rubber curtain', table).model;
  const physics = new RapierBackend({ gravity: [0, -9.81, 0], impactThreshold: 500, solverIterations: 16, pgsIterations: 1 });
  physics.createGround(0, ALL);
  const sim = new DestructionSim(model, table, physics, { client: new InlineIntegrity(model, table) });
  for (let c = 0; c < model.chunkCount; c++) sim.chains.promote(c);
  const rub = (c: number) => table.materials[model.chunkMaterial[c]].deform === 'joints', bonds: number[] = [];
  for (let b = 0; b < model.bondCount; b++) if (rub(model.bondA[b]) !== rub(model.bondB[b])) bonds.push(b);
  sim.removeBonds(bonds); // strips fall off the lintel
  const pose = new Float32Array(7);
  let minY = Infinity;
  for (let k = 0; k < 8 / h; k++) {
    physics.step(h); sim.tick(h);
    if (k % 24) continue;
    for (const isl of sim.handoff.islands) if (!isl.removed) { physics.readBodyPoses([isl.body], pose); minY = Math.min(minY, pose[1]); }
  }
  const world = (physics as unknown as { world: { bodies: { forEach(f: (b: { isSleeping(): boolean; isDynamic(): boolean }) => void): void } } }).world;
  let awake = 0;
  world.bodies.forEach((b) => { if (b.isDynamic() && !b.isSleeping()) awake++; });
  console.log(`[ground sleep] min body y ${minY.toFixed(3)} awake ${awake}`);
  expect(minY).toBeGreaterThan(0.03); // strip chunks are 0.1 thick: centre >= 0.05 on the ground
  expect(awake).toBeLessThan(10);
  physics.dispose();
}, 120000);
