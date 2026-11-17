// Main-thread core loop (NOTES Design §3, §7) for one structure: bond removals -> integrity (inline or worker) -> breaks ->
// island handoff. The frame never waits: at most one integrity request is in flight; removals queue until it returns.
// No structure solver: bonds break by scripted removal or collision fracture (CollisionBreaker) until the external solver is plugged in.

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { InlineIntegrity, IntegrityClient } from '../integrity/IntegrityClient';
import { TickReply } from '../integrity/IntegrityCore';
import { quatFromRotVec } from '../model/quat';
import { PhysicsBackend } from '../physics/PhysicsBackend';
import { IslandHandoff } from './IslandHandoff';
import { JointChainOptions, JointChains } from './JointChains';
import { BreakDecision, CollisionBreaker, CollisionBreakOptions } from './CollisionBreaker';

export interface SimOptions {
  /** Integrity backend; default inline (synchronous). Pass a WorkerIntegrity in the app. */
  client?: IntegrityClient;
  /** Joint-chain options (NOTES Design §4b). */
  chains?: JointChainOptions;
  /** Collision fracture for every contact pair (CollisionBreaker). */
  collisionBreak?: CollisionBreakOptions;
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
  /** Per bond (m,): 1 while intact (from integrity replies). */
  readonly alive: Uint8Array;
  readonly chains: JointChains;
  /** Any contact pair with enough approach energy cuts the chunks it touches and keeps its momentum. */
  readonly breaker: CollisionBreaker;
  /** Fake impact dents (NOTES Design §4 offsets) on/off; existing dents stay. */
  dents = true;
  private inFlight = false;
  private pendingRemove: number[] = [];
  /** Collision breaks decided during this step's event drain, applied after it (bodies must not change mid-drain). */
  private readonly breaks: { d: BreakDecision; bodyA: number; bodyB: number }[] = [];
  /** Impacts on static `offsets` chunks waiting for the next request (IntegrityCore IMPACT_STRIDE each). */
  private pendingImpacts: number[] = [];
  /** Min dent force (N) per chunk, Infinity = never dents (not `offsets`): (n,). Filters events before they reach the worker. */
  private readonly dentForce: Float64Array;
  private readonly px = [0, 0, 0];
  private readonly pq = [0, 0, 0, 1];

  constructor(readonly model: ChunkModel, readonly table: MaterialTable, private readonly physics: PhysicsBackend, o: SimOptions = {}) {
    this.client = o.client ?? new InlineIntegrity(model, table);
    this.handoff = new IslandHandoff(model, table, physics);
    this.brokenLog = new Uint32Array(model.bondCount);
    this.isStatic = new Uint8Array(model.chunkCount).fill(1);
    this.alive = new Uint8Array(model.bondCount).fill(1);
    this.chains = new JointChains(model, table, physics, this, o.chains);
    this.breaker = new CollisionBreaker(model, table, physics, o.collisionBreak);
    this.dentForce = new Float64Array(model.chunkCount);
    for (let i = 0; i < model.chunkCount; i++) {
      const m = table.materials[model.chunkMaterial[i]];
      this.dentForce[i] = m.deform === 'offsets' ? m.dent.minForce : Infinity;
    }
  }

  /**
   * Call once per physics step, after `physics.step(dt)`: advances grace timers, applies any integrity reply (breaks,
   * handoff), and sends queued removals when no request is in flight. @param dt step (s). @returns bonds broken.
   */
  tick(dt: number): number {
    this.handoff.update(dt);
    // impacts (Rapier auto-drains per step, so read them every substep): promote hit `joints` parts
    this.breaker.beginStep();
    this.physics.drainImpactEvents((e) => {
      // collision fracture first: a cut chunk neither dents nor promotes. Bodies change only after the drain.
      const d = this.breaker.onContact(e, dt, this.alive, this.pendingRemove);
      if (d) { this.breaks.push({ d, bodyA: e.bodyA, bodyB: e.bodyB }); return; }
      // joint-chain promotion: the part turns dynamic this step, so undo Rapier's bounce off the (still static) part
      const pa = this.chains.onImpact(e.chunkA, e.force), pb = this.chains.onImpact(e.chunkB, e.force);
      const pass = pa || pb ? this.breaker.passThrough(e, dt) : null;
      if (pass) { this.breaks.push({ d: pass, bodyA: e.bodyA, bodyB: e.bodyB }); return; }
      // dents: force on B is +dir, on A -dir; impulse = force dt
      if (!this.dents) return;
      if (e.chunkA >= 0) this.queueDent(e.chunkA, -1, e.dir, e.force, dt);
      if (e.chunkB >= 0) this.queueDent(e.chunkB, 1, e.dir, e.force, dt);
    });
    this.applyBreaks();
    this.chains.step(dt);
    this.stepsInFlight++;
    const reply = this.client.poll();
    let broke = reply ? this.apply(reply) : 0;
    if (!this.inFlight && (this.pendingRemove.length || this.pendingImpacts.length)) {
      this.send();
      // Synchronous clients answer inside request(): apply now so breaks land in the same step (no 1-step latency).
      const now = this.client.sync ? this.client.poll() : null;
      if (now) broke += this.apply(now);
    }
    return broke;
  }

  /**
   * Frees this step's collision-cut chunks now (ahead of integrity, which confirms the same components later), then
   * hands the absorbed impulse back to the pair (at the CoM); a split body's share goes to its rest piece.
   */
  private applyBreaks(): void {
    const hd = this.handoff;
    for (const k of this.breaks) {
      for (const c of k.d.chunks) {
        const id = hd.islandOf[c], old = id >= 0 ? hd.islands[id].body : -1;
        const rest = hd.splitChunk(c);
        this.isStatic[c] = 0;
        if (rest < 0) continue;
        for (const o of this.breaks) { if (o.bodyA === old) o.bodyA = rest; if (o.bodyB === old) o.bodyB = rest; }
      }
      this.breaker.handBack(k.d, k.bodyA, k.bodyB);
    }
    this.breaks.length = 0;
  }

  /** Queues scripted removals (wall removal, explosions); applied by the next integrity request. */
  removeBonds(bonds: ArrayLike<number>): void {
    for (let k = 0; k < bonds.length; k++) this.pendingRemove.push(bonds[k]);
  }

  /**
   * Queues a dent impact on static chunk c (NOTES Design §4 fake offsets) if its force reaches the material's dent.minForce.
   * Contact point = chunk centre (events carry no point). @param sign +1 / -1 applied to dir (3,). @param dt step (s).
   */
  private queueDent(c: number, sign: number, dir: ArrayLike<number>, force: number, dt: number): void {
    if (!this.isStatic[c] || force < this.dentForce[c]) return;
    const p = this.model.chunkPos;
    this.pendingImpacts.push(c, p[3 * c], p[3 * c + 1], p[3 * c + 2], sign * dir[0], sign * dir[1], sign * dir[2], force * dt, force);
  }

  private send(): void {
    const remove = Uint32Array.from(this.pendingRemove);
    this.pendingRemove.length = 0;
    const impacts = this.pendingImpacts.length ? Float64Array.from(this.pendingImpacts) : undefined;
    this.pendingImpacts.length = 0;
    this.client.request({ type: 'tick', remove, impacts });
    this.inFlight = true;
    this.stepsInFlight = 0;
  }

  /** Applies a reply: logs breaks, hands detached components to physics. @returns bonds broken. */
  private apply(r: TickReply): number {
    this.inFlight = false;
    this.replies++;
    this.replyLatencySteps = this.stepsInFlight;
    this.version = r.version;
    for (let k = 0; k < r.broken.length; k++) {
      if (!this.alive[r.broken[k]]) continue; // duplicate removal (e.g. a joint break racing a scripted cut)
      this.alive[r.broken[k]] = 0;
      this.brokenLog[this.brokenTotal++] = r.broken[k];
    }
    // dent poses first, so chunks detaching in this reply take their dented pose
    for (let k = 0; k < r.poseChunks.length; k++) {
      const c = r.poseChunks[k], o = 6 * k;
      for (let a = 0; a < 3; a++) this.px[a] = this.model.chunkPos[3 * c + a] + r.poses[o + a];
      quatFromRotVec(r.poses, o + 3, this.pq);
      if (this.handoff.islandOf[c] < 0) this.handoff.setStaticPose(c, this.px, this.pq);
    }
    const comps = r.detachedOffset.length - 1;
    if (comps > 0) {
      for (let k = 0; k < r.detachedOffset[comps]; k++) this.isStatic[r.detachedChunks[k]] = 0;
      this.handoff.detach(r.detachedChunks, r.detachedOffset, comps);
    }
    const floats = r.floatOffset.length - 1;
    if (floats > 0) this.handoff.refracture(r.floatChunks, r.floatOffset, floats);
    return r.broken.length;
  }

  dispose(): void { this.client.dispose(); }
}
