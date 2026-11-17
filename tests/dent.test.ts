// Fake impact dents for `offsets` materials (NOTES Design §4 "Fake deformation"): falloff, saturation, clamp, ring cracks, detach at the dented pose.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { IntegrityCore, IMPACT_STRIDE } from '../src/integrity/IntegrityCore';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });

const table = createMaterialTable();
const h = 0.4, dp = table.materials[materialId(table, 'concrete')].dent;
/** 5 x 5 x 1 concrete wall of 0.4 m cubes, bottom row supported; chunk index = 5 y + x, centre = 12. */
const wall = () => generateFromBoxes(gridBoxes({ counts: [5, 5, 1], cell: [h, h, h], material: materialId(table, 'concrete'), supportBottom: true }), table);

/** One impact on chunk c pushing +z with impulse J (N s) and force F (N). */
function impact(core: IntegrityCore, c: number, J: number, F = 1e6) {
  const p = core.model.chunkPos, im = new Float64Array(IMPACT_STRIDE);
  im.set([c, p[3 * c], p[3 * c + 1], p[3 * c + 2], 0, 0, 1, J, F]);
  return core.tick({ type: 'tick', remove: new Uint32Array(0), impacts: im });
}

describe('dent field', () => {
  it('moves only the damaged chunk, along the hit direction, saturating with damage points', () => {
    const core = new IntegrityCore(wall(), table), o = core.dents.offset, t = core.dents.tilt;
    impact(core, 12, 1e3, dp.minForce * 0.5);
    expect(o.every((v) => v === 0)).toBe(true);
    const r = impact(core, 12, 100);
    const s = dp.oMax * h * (1 - Math.exp(-100 / dp.jRef));
    expect(o[3 * 12 + 2]).toBeCloseTo(s, 9);
    expect(Math.hypot(o[36], o[37])).toBe(0); // displaced along the incoming direction only
    expect(Array.from(r.poseChunks)).toEqual([12]); // neighbours untouched
    for (let i = 0; i < 25; i++) if (i !== 12) expect(o[3 * i + 2]).toBe(0);
    expect(core.dents.points[12]).toBeCloseTo(100 / dp.jRef, 12);
    const th1 = Math.hypot(t[36], t[37], t[38]);
    expect(th1).toBeGreaterThan(0);
    expect(th1).toBeCloseTo(dp.tilt * (1 - Math.exp(-100 / dp.jRef)), 9); // first hit: one random axis
    // more damage -> more displaced, bounded by the clamps
    for (let k = 0; k < 20; k++) impact(core, 12, 1e4);
    expect(o[3 * 12 + 2]).toBeGreaterThan(s);
    expect(Math.hypot(o[36], o[37], o[38])).toBeLessThanOrEqual(dp.oMax * h + 1e-12);
    expect(Math.hypot(t[36], t[37], t[38])).toBeLessThanOrEqual(dp.tilt + 1e-12);
  });

  it('a hard hit never breaks bonds', () => {
    const core = new IntegrityCore(wall(), table);
    const r = impact(core, 12, 1e7);
    expect(r.broken.length).toBe(0);
    expect(r.detachedChunks.length).toBe(0);
  });
});

describe('dents in the sim', () => {
  it('a dented chunk keeps its pose when it detaches (no pop)', () => {
    const model = wall(), physics = new RapierBackend({ gravity: [0, 0, 0] });
    const sim = new DestructionSim(model, table, physics), hd = sim.handoff;
    (sim as unknown as { queueDent(c: number, s: number, d: number[], f: number, dt: number): void }).queueDent(12, 1, [0, 0, 1], 1e5, 1e-3);
    physics.step(1 / 240); sim.tick(1 / 240);
    const dz = hd.setPos[3 * 12 + 2] - model.chunkPos[3 * 12 + 2];
    expect(dz).toBeGreaterThan(0);
    expect(hd.staticPoseDirty).toContain(12);
    const ring = [] as number[];
    for (let b = 0; b < model.bondCount; b++) if (model.bondA[b] === 12 || model.bondB[b] === 12) ring.push(b);
    sim.removeBonds(ring);
    physics.step(1 / 240); sim.tick(1 / 240);
    const isl = hd.islands[hd.islandOf[12]], pose = new Float32Array(7);
    physics.readBodyPoses([isl.body], pose);
    expect(pose[2]).toBeCloseTo(model.chunkPos[3 * 12 + 2] + dz, 4);
    physics.dispose();
  });
});
