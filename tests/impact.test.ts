// Impact loads end to end (PLAN §2): ball -> contact force events -> loads -> tier 1 -> breaks/islands.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { ImpactLoads, IMPACT_AMPLIFICATION } from '../src/integrity/impactLoads';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });
const table = createMaterialTable();
const ALL = packGroups(0xffff, 0xffff);

describe('impact loads', () => {
  it('keeps the per-chunk peak, signs by side, skips non-static chunks, clears on apply', () => {
    const il = new ImpactLoads(3), isStatic = Uint8Array.of(1, 1, 0);
    il.addContact(0, -1, 10, 1, 0, 0, isStatic); // chunk A: -dir
    il.addContact(-1, 1, 5, 0, 1, 0, isStatic); // chunk B: +dir
    il.addContact(-1, 1, 3, 0, 0, 1, isStatic); // smaller: ignored
    il.addContact(-1, 2, 99, 1, 0, 0, isStatic); // debris chunk: ignored
    const f = new Float64Array(18);
    expect(il.applyAndClear(f)).toBe(2);
    expect(f[0]).toBe(-10 * IMPACT_AMPLIFICATION);
    expect(f[7]).toBe(5 * IMPACT_AMPLIFICATION);
    expect(f[8]).toBe(0);
    expect(f[12]).toBe(0);
    f.fill(0);
    expect(il.applyAndClear(f)).toBe(0);
  });

  /** Concrete wall 8x6x1 cells of 0.25 m on supports; ball of radius r, speed v, hits the middle. */
  function shoot(r: number, v: number, steps = 90) {
    const model = generateFromBoxes(gridBoxes({ counts: [8, 6, 1], cell: [0.25, 0.25, 0.25], material: materialId(table, 'concrete'), supportBottom: true }), table);
    const physics = new RapierBackend({ impactThreshold: 100 });
    physics.createGround(0, ALL);
    const sim = new DestructionSim(model, table, physics, { budgetMs: 1000 });
    physics.createSphere(r, [1, 0.9, 3], [0, 0, -v], 7850, ALL);
    let broke = 0;
    for (let k = 0; k < steps; k++) { physics.step(1 / 60); broke += sim.tick(1 / 60); }
    const out = { broke, islands: sim.handoff.islands.length };
    physics.dispose();
    return out;
  }

  it('a slow light ball does not break the wall', () => {
    const r = shoot(0.05, 2);
    expect(r.broke).toBe(0);
    expect(r.islands).toBe(0);
  });

  it('a fast heavy ball breaks bonds and knocks pieces off', () => {
    const r = shoot(0.3, 30);
    expect(r.broke).toBeGreaterThan(0);
    expect(r.islands).toBeGreaterThan(0);
  });
});
