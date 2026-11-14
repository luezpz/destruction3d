// Verifies Rapier API facts the plan relies on (PLAN "Rapier overlap notes", Risk 4). Results recorded in NOTES.md.
import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { chunkHull } from '../src/physics/fromModel';
import { GroupBit, ImpactEvent, packGroups } from '../src/physics/PhysicsBackend';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';

beforeAll(async () => { await initRapier(); });

const t = createMaterialTable();
const model = generateFromBoxes(gridBoxes({ counts: [2, 1, 1], cell: [1, 1, 1], material: materialId(t, 'concrete') }), t);
const ALL = packGroups(0xffff, 0xffff);

/** Number of contact pairs with at least one active contact point. */
function activeContacts(w: RAPIER.World): number {
  let n = 0;
  w.colliders.forEach((c1) => {
    w.narrowPhase.contactPairsWith(c1.handle, (c2) => {
      if (c2 <= c1.handle) return;
      w.narrowPhase.contactPair(c1.handle, c2, w.bodies, (m) => { if (m.numContacts() > 0) n++; });
    });
  });
  return n;
}

describe('Rapier facts', () => {
  it('Collider has no re-parent API (detach = remove + recreate)', () => {
    expect('setParent' in RAPIER.Collider.prototype).toBe(false);
  });

  it('convexMesh accepts supplied indices; setShape works', () => {
    const be = new RapierBackend();
    const body = be.createStaticBody();
    const h = be.createStaticChunk(body, chunkHull(model, 0), ALL);
    const c = chunkHull(model, 1);
    be.setChunkShape(h, c.points, c.indices);
    expect(be.world.getCollider(h).shape.type).toBe(RAPIER.ShapeType.ConvexPolyhedron);
    be.dispose();
  });

  it('overlapping colliders on one dynamic body make no contacts; island mass is explicit', () => {
    const be = new RapierBackend();
    const a = chunkHull(model, 0), b = chunkHull(model, 0); // same hull twice: full overlap
    const { body } = be.createDynamicIsland({
      position: [0, 5, 0], rotation: [0, 0, 0, 1], chunks: [a, b], mass: 123, inertia: [1, 2, 3], inertiaFrame: [0, 0, 0, 1],
      linvel: [0, 0, 0], angvel: [0, 0, 0], groups: ALL,
    });
    be.step(1 / 60);
    expect(activeContacts(be.world)).toBe(0);
    expect(be.world.getRigidBody(body).mass()).toBeCloseTo(123, 3);
    be.dispose();
  });

  it('inertiaFrame is honoured: effective world inertia tensor = R diag(I) R^T', () => {
    const be = new RapierBackend();
    const s = Math.SQRT1_2; // 90 deg about z: principal x axis -> world y
    const { body } = be.createDynamicIsland({
      position: [0, 5, 0], rotation: [0, 0, 0, 1], chunks: [chunkHull(model, 0)], mass: 10, inertia: [1, 2, 3],
      inertiaFrame: [0, 0, s, s], linvel: [0, 0, 0], angvel: [0, 0, 0], groups: ALL,
    });
    const rb = be.world.getRigidBody(body);
    be.step(1 / 60); // mass properties are only applied by a step
    // torque impulse about world y hits principal I=1 -> angvel 1; about world x hits I=2 -> 0.5
    rb.applyTorqueImpulse({ x: 1, y: 1, z: 0 }, true);
    const w = rb.angvel();
    expect(w.y).toBeCloseTo(1, 4);
    expect(w.x).toBeCloseTo(0.5, 4);
    be.dispose();
  });

  it('fixed-vs-fixed overlap makes no contacts; static chunks on one fixed body', () => {
    const be = new RapierBackend();
    const s1 = be.createStaticBody(), s2 = be.createStaticBody();
    be.createStaticChunk(s1, chunkHull(model, 0), ALL);
    be.createStaticChunk(s2, chunkHull(model, 0), ALL);
    be.step(1 / 60);
    expect(activeContacts(be.world)).toBe(0);
    be.dispose();
  });

  it('positive control: a dynamic island overlapping a static chunk (same groups) does make contacts', () => {
    const be = new RapierBackend();
    be.createStaticChunk(be.createStaticBody(), chunkHull(model, 0), ALL);
    const ch = chunkHull(model, 0);
    be.createDynamicIsland({
      position: ch.translation, rotation: [0, 0, 0, 1], chunks: [{ ...ch, translation: [0, 0, 0] }],
      mass: 10, inertia: [1, 1, 1], inertiaFrame: [0, 0, 0, 1], linvel: [0, 0, 0], angvel: [0, 0, 0], groups: ALL,
    });
    be.step(1 / 60);
    expect(activeContacts(be.world)).toBeGreaterThan(0);
    be.dispose();
  });

  it('grace group: a new island overlapping the structure is not pushed out; impact events fire on a drop', () => {
    const be = new RapierBackend({ impactThreshold: 10 });
    const s = be.createStaticBody();
    const structGroups = packGroups(GroupBit.STRUCTURE, GroupBit.STRUCTURE | GroupBit.DEBRIS);
    be.createStaticChunk(s, chunkHull(model, 0), structGroups);
    const ch = chunkHull(model, 0); // overlaps the static chunk exactly
    const island = (groups: number, y: number) => be.createDynamicIsland({
      position: [ch.translation[0], y, ch.translation[2]], rotation: [0, 0, 0, 1], chunks: [{ ...ch, translation: [0, 0, 0] }],
      mass: 2400, inertia: [400, 400, 400], inertiaFrame: [0, 0, 0, 1], linvel: [0, 0, 0], angvel: [0, 0, 0], groups,
    });
    const grace = island(packGroups(GroupBit.GRACE, GroupBit.DEBRIS), 0.5);
    be.step(1 / 60);
    expect(be.world.getRigidBody(grace.body).linvel().y).toBeCloseTo(-9.81 / 60, 3); // free fall only, no push-out
    be.removeBody(grace.body);

    island(packGroups(GroupBit.DEBRIS, GroupBit.STRUCTURE | GroupBit.DEBRIS), 2.0);
    const events: ImpactEvent[] = [];
    for (let i = 0; i < 120; i++) { be.step(1 / 60); be.drainImpactEvents((e) => events.push(e)); }
    expect(events.length).toBeGreaterThan(0);
    expect(events.some((e) => e.chunkA === 0 && e.chunkB === 0 && e.force > 0)).toBe(true);
    be.dispose();
  });

  it('impact dir points from collider A to B (force on B = +dir F), whichever was created first', () => {
    const one = generateFromBoxes(gridBoxes({ counts: [1, 1, 1], cell: [1, 1, 1], material: materialId(t, 'concrete') }), t);
    for (const chunkFirst of [true, false]) {
      const be = new RapierBackend({ impactThreshold: 1 });
      const addChunk = () => be.createStaticChunk(be.createStaticBody(), chunkHull(one, 0), ALL);
      if (chunkFirst) addChunk();
      be.createSphere(0.2, [3, 0.5, 0.5], [-20, 0, 0], 1000, ALL); // hits the +x face
      if (!chunkFirst) addChunk();
      let fx = 0;
      for (let i = 0; i < 20; i++) {
        be.step(1 / 60);
        be.drainImpactEvents((e) => { fx += (e.chunkB === 0 ? 1 : e.chunkA === 0 ? -1 : 0) * e.dir[0] * e.force; });
      }
      expect(fx).toBeLessThan(0); // the chunk is pushed towards -x
      be.dispose();
    }
  });
});
