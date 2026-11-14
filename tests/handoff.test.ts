// Tier 0 -> Rapier island handoff (PLAN §3), end to end in Node.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { BondSolver } from '../src/integrity/BondSolver';
import { StressTier } from '../src/integrity/StressTier';
import { chunkMasses, gravityLoads } from '../src/integrity/loads';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { IslandHandoff } from '../src/sim/IslandHandoff';

beforeAll(async () => { await initRapier(); });
const table = createMaterialTable();
const concrete = materialId(table, 'concrete');

/** Two columns (x=0, x=2), height 2, under a 3-wide slab at y=2..3, sitting on ground y=0. */
function portal() {
  const boxes: Box[] = [];
  for (const x of [0, 2]) for (let y = 0; y < 2; y++) boxes.push({ min: [x, y, 0], max: [x + 1, y + 1, 1], material: concrete, support: y === 0 });
  for (let x = 0; x < 3; x++) boxes.push({ min: [x, 2, 0], max: [x + 1, 3, 1], material: concrete, support: false });
  return generateFromBoxes(boxes, table);
}

describe('island handoff', () => {
  it('cutting the columns drops the slab as one body with exact mass; it lands on the column stubs', () => {
    const m = portal(), s = new BondSolver(m, table), t = new StressTier(m, table, s);
    const physics = new RapierBackend();
    physics.createGround(0, packGroups(0xffff, 0xffff));
    const h = new IslandHandoff(m, table, physics);
    const cut = [] as number[];
    for (let b = 0; b < m.bondCount; b++) if (m.bondCentroid[3 * b + 1] === 2) cut.push(b); // column-slab interfaces
    expect(cut.length).toBe(2);
    expect(t.removeBonds(cut)).toBe(1);
    const [id] = h.detach(t.detachedChunks, t.detachedOffset, t.detachedCount);
    const isl = h.islands[id];
    expect(Array.from(isl.chunks).sort()).toEqual([4, 5, 6]);
    for (const c of isl.chunks) expect(h.staticCollider[c]).toBe(-1);
    const rb = physics.world.getRigidBody(isl.body);
    physics.step(1 / 60);
    expect(rb.mass()).toBeCloseTo(3 * 2400, 0);
    for (let k = 0; k < 120; k++) { physics.step(1 / 60); h.update(1 / 60); }
    expect(isl.grace).toBeLessThanOrEqual(0);
    // slab COM starts at y = 2.5; it sinks slightly during grace, then is pushed back onto the stubs (top y = 2)
    const y = rb.translation().y;
    expect(y).toBeLessThan(2.51);
    expect(y).toBeGreaterThan(2.45);
    // static solve still converges with the detached chunks inactive
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(chunkMasses(m, table), 9.81, f);
    s.begin(f);
    expect(s.iterate(1000, 1e-8)).toBe(true);
    physics.dispose();
  });
});

