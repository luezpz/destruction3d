// Fake impact dents for `offsets` materials (PLAN §4 "Fake deformation"): falloff, saturation, clamp, ring cracks, detach at the dented pose.
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
  it('falls off with graph distance, saturates with impulse, ignores weak forces', () => {
    const core = new IntegrityCore(wall(), table), o = core.dents.offset;
    impact(core, 12, 1e3, dp.minForce * 0.5);
    expect(o.every((v) => v === 0)).toBe(true);
    const r = impact(core, 12, 100);
    const s = dp.oMax * h * (1 - Math.exp(-100 / dp.jRef));
    expect(o[3 * 12 + 2]).toBeCloseTo(s, 9);
    expect(o[3 * 13 + 2]).toBeCloseTo(s * (1 - 0.25) ** 3, 9); // one hop, R = 2: phi(1/2)
    expect(o[3 * 14 + 2]).toBe(0); // two hops = R: phi(1) = 0
    expect(Array.from(r.poseChunks).sort((a, b) => a - b)).toEqual([7, 11, 12, 13, 17]);
    expect(r.poses[6 * Array.from(r.poseChunks).indexOf(12) + 2]).toBeCloseTo(s, 9);
    expect(r.broken.length).toBe(0); // gentle hit: dent without cracks
    // tilt: neighbours lean around the hit (n x (c - p)), the hit chunk itself doesn't
    expect(Math.hypot(...core.dents.tilt.subarray(36, 39))).toBe(0);
    expect(core.dents.tilt[3 * 13 + 1]).toBeGreaterThan(0); // +x neighbour pushed +z rotates about +y
    for (let k = 0; k < 20; k++) impact(core, 11 + 10, 1e5); // far corner region, hammered: clamp holds
    for (let i = 0; i < 25; i++) expect(Math.hypot(o[3 * i], o[3 * i + 1], o[3 * i + 2])).toBeLessThanOrEqual(dp.oMax * h + 1e-12);
  });

  it('a hard hit cracks the ring around the hit chunk and punches it out', () => {
    const core = new IntegrityCore(wall(), table);
    const r = impact(core, 12, 1e5);
    const m = core.model, ring = [] as number[];
    for (let b = 0; b < m.bondCount; b++) if (m.bondA[b] === 12 || m.bondB[b] === 12) ring.push(b);
    expect(Array.from(r.broken).sort((a, b) => a - b)).toEqual(ring.sort((a, b) => a - b));
    expect(Array.from(r.detachedChunks)).toEqual([12]);
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
