// Tier 0 -> Rapier island handoff (PLAN §3), end to end in Node.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { StructureGraph } from '../src/integrity/StructureGraph';
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
    const m = portal(), t = new StructureGraph(m);
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
    physics.dispose();
  });

  it('a break inside a falling island splits it into bodies at the current pose (no pop)', () => {
    const m = portal(), t = new StructureGraph(m);
    const physics = new RapierBackend({ gravity: [0, 0, 0] });
    const h = new IslandHandoff(m, table, physics);
    const cut = [] as number[];
    for (let b = 0; b < m.bondCount; b++) if (m.bondCentroid[3 * b + 1] === 2) cut.push(b);
    t.removeBonds(cut);
    const [id] = h.detach(t.detachedChunks, t.detachedOffset, t.detachedCount);
    // spin + move the slab, then break its middle-right bond (chunks 5-6)
    const rb = physics.world.getRigidBody(h.islands[id].body);
    rb.setLinvel({ x: 1, y: 0, z: 0 }, true); rb.setAngvel({ x: 0, y: 0, z: 2 }, true);
    for (let k = 0; k < 20; k++) physics.step(1 / 60);
    const before = new Float32Array(7);
    physics.readBodyPoses([h.islands[id].body], before);
    let b56 = -1;
    for (let b = 0; b < m.bondCount; b++) if ((m.bondA[b] === 5 && m.bondB[b] === 6) || (m.bondA[b] === 6 && m.bondB[b] === 5)) b56 = b;
    t.removeBonds([b56]);
    expect(t.floatCount).toBe(2);
    const ids = h.refracture(t.floatChunks, t.floatOffset, t.floatCount);
    expect(ids.length).toBe(2);
    expect(h.islands.map((i) => Array.from(i.chunks).sort())).toEqual([[4, 5], [6]]);
    // world position of chunk 6's centre is unchanged: x_b + R_b (c - com) before == after
    const rotate = (q: ArrayLike<number>, v: number[]) => {
      const [x, y, z, w] = [q[3], q[4], q[5], q[6]];
      const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
      return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
    };
    const c6 = [m.chunkPos[18], m.chunkPos[19], m.chunkPos[20]];
    const oldCom = [1.5, 2.5, 0.5];
    const wOld = rotate(before, c6.map((v, i) => v - oldCom[i])).map((v, i) => v + before[i]);
    const isl6 = h.islands[h.islandOf[6]], after = new Float32Array(7);
    physics.readBodyPoses([isl6.body], after);
    const t6 = isl6.local.subarray(7 * h.slotOf[6], 7 * h.slotOf[6] + 3);
    const wNew = rotate(after, Array.from(t6)).map((v, i) => v + after[i]);
    for (let i = 0; i < 3; i++) expect(wNew[i]).toBeCloseTo(wOld[i], 4);
    // the piece keeps the rigid velocity field: angular velocity carried over
    const st = new Float64Array(13);
    physics.readBodyStates([isl6.body], st);
    expect(st[12]).toBeCloseTo(2, 3);
    physics.dispose();
  });
});

