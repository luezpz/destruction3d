// Integrity message boundary (NOTES Design §7): core replies, transfer round trip, version, one-in-flight client behaviour.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { IntegrityCore, TickReply, TickRequest, replyTransfer } from '../src/integrity/IntegrityCore';
import { IntegrityClient } from '../src/integrity/IntegrityClient';
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

const req = (remove: number[] = []): TickRequest => ({ type: 'tick', remove: Uint32Array.from(remove) });

describe('IntegrityCore', () => {
  it('scripted removal: broken + detached in the same reply, version bumps once per breaking request', () => {
    const m = cantilever(4, 0.2), core = new IntegrityCore(m, table);
    expect(core.tick(req()).version).toBe(0);
    const r = core.tick(req([1]));
    expect(Array.from(r.broken)).toEqual([1]);
    expect(Array.from(r.detachedOffset)).toEqual([0, 3]);
    expect(Array.from(r.detachedChunks).sort()).toEqual([2, 3, 4]);
    expect(r.version).toBe(1);
    expect(core.tick(req()).version).toBe(1);
  });

  it('survives structured-clone init and a transferred reply (worker path)', () => {
    const m = cantilever(4, 0.2), core = new IntegrityCore(structuredClone(m), structuredClone(table));
    const r = core.tick(req([0]));
    const moved = structuredClone(r, { transfer: replyTransfer(r) }) as TickReply;
    expect(r.broken.byteLength).toBe(0); // detached by the transfer
    expect(Array.from(moved.broken)).toEqual([0]);
    expect(moved.detachedOffset[1]).toBe(4);
  });
});

/** Client that holds replies until `deliver()`: simulates a slow worker. */
class DelayedClient implements IntegrityClient {
  readonly requests: TickRequest[] = [];
  readonly sync = false;
  private ready: TickReply | null = null;
  constructor(private readonly core: IntegrityCore) {}
  request(r: TickRequest): void { this.requests.push(r); }
  deliver(): void { this.ready = this.core.tick(this.requests[this.requests.length - 1]); }
  poll(): TickReply | null { const r = this.ready; this.ready = null; return r; }
  dispose(): void {}
}

describe('DestructionSim with a slow integrity client', () => {
  it('sends only with queued removals, keeps one request in flight, applies breaks + handoff on reply', () => {
    const m = cantilever(4, 0.2), physics = new RapierBackend();
    const client = new DelayedClient(new IntegrityCore(m, table));
    const sim = new DestructionSim(m, table, physics, { client });
    sim.tick(1 / 60);
    expect(client.requests.length).toBe(0); // nothing to do
    sim.removeBonds([3]);
    sim.tick(1 / 60);
    sim.removeBonds([0]);
    for (let k = 0; k < 4; k++) sim.tick(1 / 60); // reply not delivered: no new request
    expect(client.requests.length).toBe(1);
    client.deliver();
    expect(sim.tick(1 / 60)).toBe(1); // applies reply 1 (tip falls), sends the queued removal
    expect(sim.handoff.islands.length).toBe(1);
    expect(client.requests.length).toBe(2);
    expect(Array.from(client.requests[1].remove)).toEqual([0]);
    client.deliver();
    expect(sim.tick(1 / 60)).toBe(1);
    expect(sim.brokenTotal).toBe(2);
    expect(sim.handoff.islands.length).toBe(2);
    for (let i = 1; i <= 4; i++) expect(sim.isStatic[i]).toBe(0);
    physics.dispose();
  });
});
