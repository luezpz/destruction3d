// Joint-chain demo scenes (steel/rubber): promoted at rest they hold their own weight without yielding, and a ball hit bends them.
import { beforeAll, describe, expect, it } from 'vitest';
import { MATERIAL_PRESETS, UNIT_SCALE, createMaterialTable, scaleMaterials } from '../src/model/Material';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';
import { InlineIntegrity } from '../src/integrity/IntegrityClient';
import { SCENE_STRENGTH, SceneName, buildScene } from '../src/scenes';

beforeAll(async () => { await initRapier(); });

const JOINT_SCENES: SceneName[] = ['steel cantilever', 'steel beam', 'steel truss bridge', 'steel frame', 'steel tower', 'steel pole', 'rubber curtain'];
const ALL = packGroups(0xffff, 0xffff), h = 1 / 240;

/** Builds a joint scene with its default strengths and a sim with demotion off. */
function setup(name: SceneName) {
  const per = SCENE_STRENGTH[name] ?? {};
  const table = createMaterialTable(scaleMaterials(MATERIAL_PRESETS, UNIT_SCALE, per));
  const demo = buildScene(name, table), model = demo.model;
  const physics = new RapierBackend({ gravity: [0, -9.81, 0], impactThreshold: 500, solverIterations: 16, pgsIterations: 1 });
  physics.createGround(0, ALL);
  const sim = new DestructionSim(model, table, physics, { client: new InlineIntegrity(model, table), chains: { demoteTime: Infinity } });
  const run = (seconds: number) => { for (let k = 0; k < Math.round(seconds / h); k++) { physics.step(h); sim.tick(h); } };
  return { demo, model, physics, sim, run };
}

/** Largest chunk-body drop (m) since promotion and the number of plastic bonds. */
function metrics(s: ReturnType<typeof setup>, y0: Float64Array) {
  const chains = s.sim.chains, pose = new Float32Array(7);
  const priv = chains as unknown as { pv: { epsP: Float64Array }; bodyChunk: number[] }; // test-only peeks at private state
  let drop = 0, maxMove = 0;
  for (let i = 1; i < chains.bodies.length; i++) {
    s.physics.readBodyPoses([chains.bodies[i]], pose);
    const c = priv.bodyChunk[i];
    drop = Math.max(drop, y0[c] - pose[1]);
    maxMove = Math.max(maxMove, Math.hypot(pose[0] - s.model.chunkPos[3 * c], pose[1] - y0[c], pose[2] - s.model.chunkPos[3 * c + 2]));
  }
  const epsP = priv.pv.epsP; // live per joint (bondEpsP only syncs on demotion)
  let plastic = 0;
  for (let j = 0; j < chains.jointCount; j++) if (epsP[j] > 0) plastic++;
  return { drop, maxMove, plastic, broken: chains.brokenJoints };
}

describe('joint demo scenes', () => {
  for (const name of JOINT_SCENES) {
    it(`${name}: holds its own weight, bends under a ball`, () => {
      const s = setup(name), { chains } = s.sim, { model } = s;
      const y0 = Float64Array.from({ length: model.chunkCount }, (_, c) => model.chunkPos[3 * c + 1]);
      let n = 0;
      for (let c = 0; c < model.chunkCount; c++) n += chains.promote(c); // every part (curtain strips are separate parts)
      s.run(3);
      const rest = metrics(s, y0);
      const cam = s.demo.camera, t = s.demo.target, d = [t[0] - cam[0], t[1] - cam[1], t[2] - cam[2]], l = Math.hypot(...d);
      d[1] += 0.5 * 9.81 * (l / 20) ** 2; // gravity lead, as the app's cursor throws
      s.physics.createSphere(0.25, [...cam] as [number, number, number], d.map((x) => 20 * x / Math.hypot(...d)) as [number, number, number], 7850, ALL);
      s.run(4);
      const hit = metrics(s, y0);
      console.log(`[scene ${name}] bodies ${n} joints ${chains.jointCount} | rest: drop ${rest.drop.toFixed(3)} plastic ${rest.plastic} broken ${rest.broken} | hit: drop ${hit.drop.toFixed(3)} move ${hit.maxMove.toFixed(2)} plastic ${hit.plastic} broken ${hit.broken}`);
      expect(n).toBeGreaterThan(10);
      expect(rest.plastic).toBe(0); // holds its own weight elastically
      expect(rest.broken).toBe(0);
      expect(rest.drop).toBeLessThan(0.1);
      if (name.startsWith('steel')) expect(hit.plastic).toBeGreaterThan(0); // the ball bends it permanently
      else expect(hit.maxMove).toBeGreaterThan(0.05); // soft material swings
      s.physics.dispose();
    }, 120000);
  }
});
