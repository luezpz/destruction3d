// Joint parts cut loose before promotion (NOTES Design §4b): a static `joints` part detached with its non-joint support becomes
// a free joint chain (bendable, realistic damping), not a stiff rigid island, still jointed to its detached support piece.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });

describe('joint part detached while static', () => {
  it('becomes a joint chain with free damping', () => {
    const table = createMaterialTable(), h = 0.1, boxes: Box[] = [];
    const concrete = materialId(table, 'concrete'), steel = materialId(table, 'steel');
    // chunk 0 support (concrete), chunk 1 concrete, chunks 2..6 steel cantilever off chunk 1
    for (let k = 0; k < 7; k++) boxes.push({ min: [k * h, 1, 0], max: [(k + 1) * h, 1 + h, h], material: k < 2 ? concrete : steel, support: k === 0 });
    const model = generateFromBoxes(boxes, table);
    const physics = new RapierBackend({ solverIterations: 16 });
    const sim = new DestructionSim(model, table, physics, { chains: { promoteForce: Infinity, demoteTime: Infinity } });
    const damp: number[][] = [];
    const set = physics.setBodyDamping.bind(physics);
    physics.setBodyDamping = (b, l, a) => { damp.push([l, a]); set(b, l, a); };
    const cut: number[] = [];
    for (let b = 0; b < model.bondCount; b++) if (model.bondA[b] === 0 || model.bondB[b] === 0) cut.push(b);
    sim.removeBonds(cut);
    for (let s = 0; s < 60; s++) { physics.step(1 / 240); sim.tick(1 / 240); }
    for (let c = 2; c < 7; c++) expect(sim.chains.chunkBody[c]).toBeGreaterThanOrEqual(0);
    expect(sim.chains.chunkBody[1]).toBe(-1); // concrete: rigid island
    expect(sim.handoff.islandOf[1]).toBeGreaterThanOrEqual(0);
    expect(sim.chains.aliveJoints).toBe(5); // 4 steel-steel + the anchor re-bound onto the detached concrete island
    expect(sim.chains.rebound).toBe(1);
    expect(damp.at(-1)).toEqual([0, 0.05]);
    physics.dispose();
  });

  it('a fragment broken off a still-anchored part gets free damping, the anchored rest keeps settle damping', () => {
    const table = createMaterialTable(), h = 0.1, boxes: Box[] = [], steel = materialId(table, 'steel'), n = 8;
    for (let k = 0; k < n; k++) boxes.push({ min: [k * h, 1, 0], max: [(k + 1) * h, 1 + h, h], material: steel, support: k === 0 });
    const model = generateFromBoxes(boxes, table);
    const physics = new RapierBackend({ solverIterations: 16 });
    const sim = new DestructionSim(model, table, physics, { chains: { promoteForce: Infinity, demoteTime: Infinity } });
    sim.chains.promote(1);
    const damp = new Map<number, number[]>();
    const set = physics.setBodyDamping.bind(physics);
    physics.setBodyDamping = (b, l, a) => { damp.set(b, [l, a]); set(b, l, a); };
    let mid = -1; // bond between chunks 4 and 5
    for (let b = 0; b < model.bondCount; b++) if (model.bondA[b] + model.bondB[b] === 9) mid = b;
    sim.removeBonds([mid]);
    for (let s = 0; s < 8; s++) { physics.step(1 / 240); sim.tick(1 / 240); }
    const body = (c: number) => sim.chains.bodies[sim.chains.chunkBody[c]];
    for (let c = 5; c < n; c++) expect(damp.get(body(c))).toEqual([0, 0.05]); // jointed fragment, no anchor
    for (let c = 1; c < 5; c++) expect(damp.has(body(c))).toBe(false); // still anchored: untouched settle damping
    physics.dispose();
  });
});
