// Integrity message boundary (PLAN §7): core replies, transfer round trip, version, one-in-flight client behaviour.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { IntegrityCore, TickReply, TickRequest, replyTransfer } from '../src/integrity/IntegrityCore';
import { IntegrityClient } from '../src/integrity/IntegrityClient';
import { chunkMasses, gravityLoads } from '../src/integrity/loads';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { DestructionSim } from '../src/sim/DestructionSim';

beforeAll(async () => { await initRapier(); });
const table = createMaterialTable();
const concrete = materialId(table, 'concrete');

/** Horizontal chain of N+1 cubes (side h), chunk 0 support. */
function cantilever(N: number, h: number) {
  const boxes: Box[] = [];
  for (let k = 0; k <= N; k++) boxes.push({ min: [k * h, 0, 0], max: [(k + 1) * h, h, h], material: concrete, support: k === 0 });
  return generateFromBoxes(boxes, table);
}

const req = (f: Float64Array, remove: number[] = [], dt = 1 / 60): TickRequest =>
  ({ type: 'tick', dt, f, remove: Uint32Array.from(remove), budgetMs: 1000, tol: 1e-8 });

describe('IntegrityCore', () => {
  it('returns the load buffer, ratios, and survives a structured-clone transfer (worker path)', () => {
    const m = cantilever(4, 0.2), core = new IntegrityCore(structuredClone(m), structuredClone(table));
    const f = new Float64Array(6 * m.chunkCount);
    gravityLoads(chunkMasses(m, table), 9.81, f);
    const r = core.tick(req(f));
    expect(r.f).toBe(f);
    expect(r.converged).toBe(true);
    expect(r.ratio.length).toBe(m.bondCount);
    expect(r.ratio[0]).toBeGreaterThan(0); // root bond carries the self-weight moment; filtered ratio rising
    const moved = structuredClone(r, { transfer: replyTransfer(r) }) as TickReply;
    expect(f.byteLength).toBe(0); // detached by the transfer
    expect(moved.f.length).toBe(6 * m.chunkCount);
  });

  it('scripted removal: broken + detached in the same reply, version bumps once per breaking request', () => {
    const m = cantilever(4, 0.2), core = new IntegrityCore(m, table);
    const f = new Float64Array(6 * m.chunkCount);
    expect(core.tick(req(f)).version).toBe(0);
    const r = core.tick(req(f, [1]));
    expect(Array.from(r.broken)).toEqual([1]);
    expect(Array.from(r.detachedOffset)).toEqual([0, 3]);
    expect(Array.from(r.detachedChunks).sort()).toEqual([2, 3, 4]);
    expect(r.version).toBe(1);
    expect(core.tick(req(f)).version).toBe(1);
  });
});

/** Client that holds replies until `deliver()`: simulates a slow worker. */
class DelayedClient implements IntegrityClient {
  readonly requests: TickRequest[] = [];
  private ready: TickReply | null = null;
  constructor(private readonly core: IntegrityCore) {}
  request(r: TickRequest): void { this.requests.push(r); }
  deliver(): void { this.ready = this.core.tick(this.requests[this.requests.length - 1]); }
  poll(): TickReply | null { const r = this.ready; this.ready = null; return r; }
  dispose(): void {}
}

describe('DestructionSim with a slow integrity client', () => {
  it('keeps one request in flight, accumulates dt, applies breaks + handoff when the reply lands', () => {
    const m = cantilever(4, 0.2), physics = new RapierBackend();
    const client = new DelayedClient(new IntegrityCore(m, table));
    const sim = new DestructionSim(m, table, physics, { client });
    sim.tick(1 / 60);
    sim.removeBonds([0]);
    for (let k = 0; k < 4; k++) sim.tick(1 / 60); // reply not delivered: no new request
    expect(client.requests.length).toBe(1);
    client.deliver();
    sim.tick(1 / 60); // applies reply 1 (no breaks), sends request 2 with the removal and the accumulated dt
    expect(client.requests.length).toBe(2);
    expect(client.requests[1].dt).toBeCloseTo(5 / 60, 12);
    expect(Array.from(client.requests[1].remove)).toEqual([0]);
    client.deliver();
    expect(sim.tick(1 / 60)).toBe(1);
    expect(sim.brokenTotal).toBe(1);
    expect(sim.handoff.islands.length).toBe(1);
    for (let i = 1; i <= 4; i++) expect(sim.isStatic[i]).toBe(0);
    physics.dispose();
  });
});
