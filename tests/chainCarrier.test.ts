// Joint chains re-bound to a falling rigid island (NOTES Design §4b "carriers"): rubber curtain strips stay attached to the
// lintel after its supports are cut, both when the strips were already promoted and when they were still static.
import { beforeAll, describe, expect, it } from 'vitest';
import { MATERIAL_PRESETS, UNIT_SCALE, createMaterialTable, scaleMaterials } from '../src/model/Material';
import { ChunkFlag } from '../src/model/ChunkModel';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';
import { InlineIntegrity } from '../src/integrity/IntegrityClient';
import { SCENE_STRENGTH, buildScene } from '../src/scenes';

beforeAll(async () => { await initRapier(); });

const ALL = packGroups(0xffff, 0xffff), h = 1 / 240;

/** Curtain scene + sim; `cut()` removes every bond touching a support chunk, so the whole frame falls. */
function setup() {
  const table = createMaterialTable(scaleMaterials(MATERIAL_PRESETS, UNIT_SCALE, SCENE_STRENGTH['rubber curtain'] ?? {}));
  const model = buildScene('rubber curtain', table).model;
  const physics = new RapierBackend({ gravity: [0, -9.81, 0], impactThreshold: 500, solverIterations: 16, pgsIterations: 1 });
  physics.createGround(0, ALL);
  const sim = new DestructionSim(model, table, physics, { client: new InlineIntegrity(model, table), chains: { demoteTime: Infinity } });
  const run = (seconds: number) => { for (let k = 0; k < Math.round(seconds / h); k++) { physics.step(h); sim.tick(h); } };
  const cut = () => {
    const sup = (c: number) => (model.chunkFlags[c] & ChunkFlag.SUPPORT) !== 0, bonds: number[] = [];
    for (let b = 0; b < model.bondCount; b++) if (sup(model.bondA[b]) !== sup(model.bondB[b])) bonds.push(b);
    sim.removeBonds(bonds);
  };
  return { model, physics, sim, run, cut };
}

describe('joint chains on a falling carrier', () => {
  for (const promoted of [true, false]) {
    it(`curtain strips stay attached after collapse (strips ${promoted ? 'promoted' : 'static'} before the cut)`, () => {
      const s = setup(), ch = s.sim.chains;
      if (promoted) { for (let c = 0; c < s.model.chunkCount; c++) ch.promote(c); s.run(1); }
      s.cut();
      s.run(1.5);
      console.log(`[carrier ${promoted}] rebound ${ch.rebound} alive ${ch.aliveJoints}/${ch.jointCount} broken ${ch.brokenJoints}`);
      expect(ch.rebound).toBeGreaterThanOrEqual(27); // 3 lintel bonds per strip; more when landing splits the lintel island
      expect(ch.brokenJoints).toBe(0); // no joint fractured or dropped for losing its anchor
      expect(ch.aliveJoints).toBeGreaterThan(ch.jointCount - 6); // only bonds of lintel chunks cut by the landing impact go
      // every joint still holds: anchor gap (translation lock residual) small
      for (let b = 0; b < s.model.bondCount; b++) {
        const st = ch.jointState(b);
        if (st?.alive) expect(st.gap).toBeLessThan(0.05);
      }
      s.physics.dispose();
    }, 120000);
  }
});
