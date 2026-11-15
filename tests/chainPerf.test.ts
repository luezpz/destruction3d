// Joint-chain settling + cost (PLAN §7 "measure first"; numbers in NOTES "Joint-chain perf"): a hit 3 cm steel plate
// (90 bodies, 171 joints) must come to rest and sleep quickly, since an awake chain costs ~10+ ms/frame in Rapier.
import { beforeAll, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });

it('hit steel plate settles and sleeps within 3 s (16 iterations, 4 substeps)', () => {
  const t = createMaterialTable();
  const model = generateFromBoxes(gridBoxes({ counts: [10, 10, 1], cell: [0.2, 0.2, 0.03], origin: [-1, 0, -0.015], material: materialId(t, 'steel'), supportBottom: true }), t);
  const physics = new RapierBackend({ solverIterations: 16 });
  const sim = new DestructionSim(model, t, physics, { chains: { promoteForce: Infinity, demoteTime: Infinity } });
  expect(sim.chains.promote(15)).toBe(90);
  // impact-like push on the top middle chunks
  for (const c of [85, 94, 95]) physics.world.getRigidBody(sim.chains.bodies[sim.chains.chunkBody[c]]).setLinvel({ x: 0, y: 0, z: -6 }, true);
  const ms: number[] = [];
  for (let s = 0; s < 4; s++) {
    let tot = 0;
    for (let f = 0; f < 60; f++) for (let k = 0; k < 4; k++) { const a = performance.now(); physics.step(1 / 240); sim.tick(1 / 240); tot += performance.now() - a; }
    ms.push(tot / 60);
  }
  const awake = sim.chains.bodies.slice(1).filter((h) => !physics.world.getRigidBody(h).isSleeping()).length;
  console.log(`[chain perf] ms/frame per second: ${ms.map((x) => x.toFixed(1)).join(' ')}; awake after 4 s: ${awake}`);
  expect(awake).toBe(0);
  expect(sim.chains.brokenJoints).toBe(0);
  physics.dispose();
});
