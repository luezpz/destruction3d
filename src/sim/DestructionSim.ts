// Main-thread core loop (PLAN §2, §3, §7) for one structure: physics impacts -> loads -> integrity (inline or worker)
// -> breaks -> island handoff. The frame never waits: at most one integrity request is in flight; impacts and dt
// accumulate until its reply arrives. Load buffers are double-buffered (one in flight, one returned spare).

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { ImpactLoads } from '../integrity/impactLoads';
import { chunkMasses, gravityLoads } from '../integrity/loads';
import { InlineIntegrity, IntegrityClient } from '../integrity/IntegrityClient';
import { TickReply, TickStats } from '../integrity/IntegrityCore';
import { PhysicsBackend } from '../physics/PhysicsBackend';
import { IslandHandoff } from './IslandHandoff';

export interface SimOptions {
  g?: number; // m/s^2
  budgetMs?: number; // solver time per request
  tol?: number; // tier-1 PCG tolerance (NOTES: ~1e-4)
  impactHold?: number; // s an impact peak stays in the loads (~ material filterTime, default 0.05)
  /** Integrity backend; default inline (synchronous). Pass a WorkerIntegrity in the app. */
  client?: IntegrityClient;
}

export class DestructionSim {
  readonly client: IntegrityClient;
  readonly impacts: ImpactLoads;
  /**
   * Impact peaks already sent, re-sent until a converged solve has used them AND they are `impactHold` s old: a peak
   * seen by only one tier tick is attenuated ~dt/filterTime by the ratio low-pass, so ball hits never broke anything.
   */
  private readonly heldImpacts: ImpactLoads;
  readonly handoff: IslandHandoff;
  /** Bonds broken since the sim started (cumulative list for renderers): brokenLog[0 .. brokenTotal). */
  readonly brokenLog: Uint32Array;
  brokenTotal = 0;
  /** Latest filtered failure ratio per bond (m,) and graph version from integrity. */
  ratio: Float32Array;
  version = 0;
  /** Latest damage per bond (m,) in [0, 1] and solver diagnostics of the last reply (null before the first). */
  damage: Float32Array;
  lastStats: TickStats | null = null;
  /** Debug counters: replies applied, converged replies, physics steps the last reply took, chunks with held impacts. */
  replies = 0;
  convergedReplies = 0;
  replyLatencySteps = 0;
  heldChunks = 0;
  private stepsInFlight = 0;
  /** Per chunk (n,): 1 while part of the static structure (impacts only load these). */
  readonly isStatic: Uint8Array;
  private readonly gravity: Float64Array;
  private spare: Float64Array | null;
  private inFlight = false;
  private pendingDt = 0;
  private pendingRemove: number[] = [];
  /** Tunables (live-editable): solver ms per request, PCG tolerance, impact hold (s). */
  budgetMs: number;
  tol: number;
  impactHold: number;

  constructor(readonly model: ChunkModel, table: MaterialTable, private readonly physics: PhysicsBackend, o: SimOptions = {}) {
    this.client = o.client ?? new InlineIntegrity(model, table);
    this.impacts = new ImpactLoads(model.chunkCount);
    this.heldImpacts = new ImpactLoads(model.chunkCount);
    this.handoff = new IslandHandoff(model, table, physics);
    this.brokenLog = new Uint32Array(model.bondCount);
    this.ratio = new Float32Array(model.bondCount);
    this.damage = new Float32Array(model.bondCount);
    this.isStatic = new Uint8Array(model.chunkCount).fill(1);
    this.gravity = new Float64Array(6 * model.chunkCount);
    gravityLoads(chunkMasses(model, table), o.g ?? 9.81, this.gravity);
    this.spare = new Float64Array(6 * model.chunkCount);
    this.budgetMs = o.budgetMs ?? 2;
    this.tol = o.tol ?? 1e-4;
    this.impactHold = o.impactHold ?? 0.05;
  }

  /**
   * Call once per physics step, after `physics.step(dt)`: drains impacts, applies any integrity reply (breaks,
   * handoff), and sends the next request when none is in flight. @returns bonds broken by the applied reply.
   */
  tick(dt: number): number {
    this.physics.drainImpactEvents((e) => this.impacts.addContact(e.chunkA, e.chunkB, e.force, e.dir[0], e.dir[1], e.dir[2], this.isStatic));
    this.handoff.update(dt);
    this.heldImpacts.advance(dt);
    this.pendingDt += dt;
    this.stepsInFlight++;
    const reply = this.client.poll();
    let broke = reply ? this.apply(reply) : 0;
    if (!this.inFlight) {
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
    const f = this.spare!;
    this.spare = null;
    f.set(this.gravity);
    this.impacts.moveInto(this.heldImpacts);
    this.heldChunks = this.heldImpacts.apply(f);
    const remove = Uint32Array.from(this.pendingRemove);
    this.pendingRemove.length = 0;
    this.client.request({ type: 'tick', dt: this.pendingDt, f, remove, budgetMs: this.budgetMs, tol: this.tol });
    this.pendingDt = 0;
    this.inFlight = true;
    this.stepsInFlight = 0;
  }

  /** Applies a reply: logs breaks, hands detached components to physics. @returns bonds broken. */
  private apply(r: TickReply): number {
    this.inFlight = false;
    if (r.converged) this.heldImpacts.dropOlderThan(this.impactHold);
    this.spare = r.f;
    this.ratio = r.ratio;
    this.damage = r.damage;
    this.lastStats = r.stats;
    this.replies++;
    if (r.converged) this.convergedReplies++;
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
