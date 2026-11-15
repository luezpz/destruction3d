// Main-thread core loop (PLAN §3, §7) for one structure: bond removals -> integrity (inline or worker) -> breaks ->
// island handoff. The frame never waits: at most one integrity request is in flight; removals queue until it returns.
// No structure solver: bonds break only by scripted removal until the external solver is plugged in.

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { InlineIntegrity, IntegrityClient } from '../integrity/IntegrityClient';
import { TickReply } from '../integrity/IntegrityCore';
import { PhysicsBackend } from '../physics/PhysicsBackend';
import { IslandHandoff } from './IslandHandoff';

export interface SimOptions {
  /** Integrity backend; default inline (synchronous). Pass a WorkerIntegrity in the app. */
  client?: IntegrityClient;
}

export class DestructionSim {
  readonly client: IntegrityClient;
  readonly handoff: IslandHandoff;
  /** Bonds broken since the sim started (cumulative list for renderers): brokenLog[0 .. brokenTotal). */
  readonly brokenLog: Uint32Array;
  brokenTotal = 0;
  /** Graph version of the last applied reply. */
  version = 0;
  /** Debug counters: replies applied, physics steps the last reply took. */
  replies = 0;
  replyLatencySteps = 0;
  private stepsInFlight = 0;
  /** Per chunk (n,): 1 while part of the static structure. */
  readonly isStatic: Uint8Array;
  private inFlight = false;
  private pendingRemove: number[] = [];

  constructor(readonly model: ChunkModel, table: MaterialTable, physics: PhysicsBackend, o: SimOptions = {}) {
    this.client = o.client ?? new InlineIntegrity(model, table);
    this.handoff = new IslandHandoff(model, table, physics);
    this.brokenLog = new Uint32Array(model.bondCount);
    this.isStatic = new Uint8Array(model.chunkCount).fill(1);
  }

  /**
   * Call once per physics step, after `physics.step(dt)`: advances grace timers, applies any integrity reply (breaks,
   * handoff), and sends queued removals when no request is in flight. @param dt step (s). @returns bonds broken.
   */
  tick(dt: number): number {
    this.handoff.update(dt);
    this.stepsInFlight++;
    const reply = this.client.poll();
    let broke = reply ? this.apply(reply) : 0;
    if (!this.inFlight && this.pendingRemove.length) {
      this.send();
      // Synchronous clients answer inside request(): apply now so breaks land in the same step (no 1-step latency).
      const now = this.client.sync ? this.client.poll() : null;
      if (now) broke += this.apply(now);
    }
    return broke;
  }

  /** Queues scripted removals (wall removal, explosions); applied by the next integrity request. */
  removeBonds(bonds: ArrayLike<number>): void {
    for (let k = 0; k < bonds.length; k++) this.pendingRemove.push(bonds[k]);
  }

  private send(): void {
    const remove = Uint32Array.from(this.pendingRemove);
    this.pendingRemove.length = 0;
    this.client.request({ type: 'tick', remove });
    this.inFlight = true;
    this.stepsInFlight = 0;
  }

  /** Applies a reply: logs breaks, hands detached components to physics. @returns bonds broken. */
  private apply(r: TickReply): number {
    this.inFlight = false;
    this.replies++;
    this.replyLatencySteps = this.stepsInFlight;
    this.version = r.version;
    for (let k = 0; k < r.broken.length; k++) this.brokenLog[this.brokenTotal++] = r.broken[k];
    const comps = r.detachedOffset.length - 1;
    if (comps > 0) {
      for (let k = 0; k < r.detachedOffset[comps]; k++) this.isStatic[r.detachedChunks[k]] = 0;
      this.handoff.detach(r.detachedChunks, r.detachedOffset, comps);
    }
    return r.broken.length;
  }

  dispose(): void { this.client.dispose(); }
}
