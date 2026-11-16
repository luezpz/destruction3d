// Island handoff (PLAN §3): detached components from tier 0 -> dynamic physics bodies.
// Removes the chunks' static colliders, creates one body per component at its COM with explicit mass/inertia,
// in the grace group (overlaps the structure without push-out) until `graceTime` passes, then normal debris groups.
// Also the reverse (`attach`, joint-chain demotion): islands back to static colliders at their current pose.
// Talks to physics only through PhysicsBackend.

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { quatMul, quatRotate } from '../model/quat';
import { chunkMasses, chunkRotInertia } from '../integrity/loads';
import { IslandMass, islandMassProperties } from '../integrity/islandMass';
import { BodyHandle, ColliderHandle, GroupBit, IslandChunk, PhysicsBackend, packGroups } from '../physics/PhysicsBackend';
import { chunkHull } from '../physics/fromModel';

export const STRUCTURE_GROUPS = packGroups(GroupBit.STRUCTURE, 0xffff & ~GroupBit.GRACE);
export const GRACE_GROUPS = packGroups(GroupBit.GRACE, GroupBit.DEBRIS);
export const DEBRIS_GROUPS = packGroups(GroupBit.DEBRIS, 0xffff);

export interface Island {
  body: BodyHandle;
  colliders: ColliderHandle[];
  chunks: Uint32Array;
  /**
   * Pose of each chunk in the body frame, (7k,): centre t (3), rotation r (4, quat xyzw).
   * World chunk pose = body * local: x_c = x_b + q_b t, q_c = q_b r. Undeformed islands: t = chunkPos - COM, r = 1.
   */
  local: Float64Array;
  /** Seconds left in the grace group (<= 0: normal debris). */
  grace: number;
  /** Groups after grace (default DEBRIS_GROUPS); joint-chain parts use a part bit so a part doesn't collide with itself. */
  groups?: number;
  /** True once re-attached to the static structure (`attach`); the entry stays so island indices are stable. */
  removed?: boolean;
}

export class IslandHandoff {
  readonly staticBody: BodyHandle;
  /** Static collider of each chunk, -1 once handed off: (n,). Float64: Rapier handles are non-integer doubles. */
  readonly staticCollider: Float64Array;
  /** Island index of each chunk, -1 = static structure: (n,). */
  readonly islandOf: Int32Array;
  /** Index of each chunk inside its island's `chunks`/`local` (valid while islandOf >= 0): (n,). */
  readonly slotOf: Int32Array;
  /**
   * Static pose of each chunk (structure frame): centre setPos (3n,), rotation setQ (4n,). Rest pose until a deformed
   * joint-chain part is demoted (PLAN §4b), then its frozen shape; new islands start from it so nothing pops.
   */
  readonly setPos: Float64Array;
  readonly setQ: Float64Array;
  /** Chunks whose static pose changed; the renderer writes their skin pose once and clears the list. */
  readonly staticPoseDirty: number[] = [];
  readonly islands: Island[] = [];
  private readonly mass: Float64Array;
  private readonly rotInertia: Float64Array;
  /** Scratch chunk positions (3n,) for mass properties in a body frame. */
  private readonly work: Float64Array;

  /** Builds the static structure body with one collider per chunk. @param graceTime s in the grace group after detaching. */
  constructor(private readonly model: ChunkModel, table: MaterialTable, private readonly physics: PhysicsBackend,
    private readonly graceTime = 0.1) { // short: long enough to separate rest overlaps, short enough not to fall through ledges
    const n = model.chunkCount;
    this.mass = chunkMasses(model, table);
    this.rotInertia = chunkRotInertia(model, this.mass);
    this.staticBody = physics.createStaticBody();
    this.staticCollider = new Float64Array(n);
    this.islandOf = new Int32Array(n).fill(-1);
    this.slotOf = new Int32Array(n).fill(-1);
    this.setPos = Float64Array.from(model.chunkPos.subarray(0, 3 * n));
    this.setQ = new Float64Array(4 * n);
    for (let i = 0; i < n; i++) this.setQ[4 * i + 3] = 1;
    this.work = new Float64Array(3 * n);
    for (let i = 0; i < n; i++) this.staticCollider[i] = physics.createStaticChunk(this.staticBody, chunkHull(model, i), STRUCTURE_GROUPS);
  }

  /**
   * Hands off components chunks[offset[k]..offset[k+1]) for k < count (Connectivity output layout).
   * Velocity 0: pieces leave the static structure (PLAN §3 step 3). @returns indices of the new islands.
   */
  detach(chunks: Uint32Array, offset: Uint32Array, count: number): number[] {
    const created: number[] = [];
    for (let k = 0; k < count; k++) {
      // skip chunks already on bodies (debris, or promoted joint-chain chunks); splits of those go through refracture()
      let any = false;
      for (let q = offset[k]; q < offset[k + 1]; q++) if (this.islandOf[chunks[q]] < 0) { any = true; break; }
      if (!any) continue;
      let own = chunks.subarray(offset[k], offset[k + 1]);
      if (own.some((c) => this.islandOf[c] >= 0)) own = own.filter((c) => this.islandOf[c] < 0);
      this.detachOne(own, created);
    }
    return created;
  }

  /**
   * Hands one component (all static chunks) to a new dynamic island at the chunks' static poses; pushes its index to
   * `created`. A single chunk's body takes the chunk's rotation (body frame = chunk frame, which joint chains rely on).
   */
  private detachOne(comp: Uint32Array, created: number[]): void {
    for (const c of comp) {
      if (this.staticCollider[c] !== -1) { this.physics.removeCollider(this.staticCollider[c]); this.staticCollider[c] = -1; }
    }
    const mp = islandMassProperties(comp, 0, comp.length, this.mass, this.rotInertia, this.setPos);
    const single = comp.length === 1, local = new Float64Array(7 * comp.length);
    comp.forEach((c, k) => {
      for (let i = 0; i < 3; i++) local[7 * k + i] = single ? 0 : this.setPos[3 * c + i] - mp.com[i];
      for (let i = 0; i < 4; i++) local[7 * k + 3 + i] = single ? (i === 3 ? 1 : 0) : this.setQ[4 * c + i];
    });
    const q = single ? this.setQ.subarray(4 * comp[0], 4 * comp[0] + 4) : [0, 0, 0, 1];
    this.createIsland(comp, local, mp, [mp.com[0], mp.com[1], mp.com[2]], [q[0], q[1], q[2], q[3]], [0, 0, 0], [0, 0, 0], -1, created);
  }

  /**
   * Creates an island body (origin `position`, rotation quat `rotation`) with chunk colliders at `local` (7k,) and registers
   * it at island index `reuse` (-1 = append); pushes the index to `out`. @param mp mass/inertia in the body frame.
   */
  private createIsland(chunks: Uint32Array, local: Float64Array, mp: IslandMass, position: [number, number, number],
    rotation: [number, number, number, number], linvel: [number, number, number], angvel: [number, number, number],
    reuse: number, out: number[]): void {
    const list: IslandChunk[] = [];
    chunks.forEach((c, k) => {
      const h = chunkHull(this.model, c);
      h.translation = [local[7 * k], local[7 * k + 1], local[7 * k + 2]];
      if (local[7 * k + 6] !== 1) h.rotation = [local[7 * k + 3], local[7 * k + 4], local[7 * k + 5], local[7 * k + 6]];
      list.push(h);
    });
    const { body, colliders } = this.physics.createDynamicIsland({
      position, rotation, chunks: list, mass: mp.mass, inertia: mp.inertia, inertiaFrame: mp.frame, linvel, angvel, groups: GRACE_GROUPS,
    });
    const id = reuse >= 0 ? reuse : this.islands.length;
    const isl: Island = { body, colliders, chunks: chunks.slice(), local, grace: this.graceTime };
    if (reuse >= 0) this.islands[id] = isl; else this.islands.push(isl);
    isl.chunks.forEach((c, k) => { this.islandOf[c] = id; this.slotOf[c] = k; });
    out.push(id);
  }

  /**
   * Frees chunk c now, ahead of integrity (collision fracture: all its bonds are being cut, so it is its own component):
   * a static chunk becomes a resting single-chunk island; an island chunk splits off (rest = one piece reusing the
   * island index; integrity's later reply refines the rest). @returns body of the rest after the split (the old body's
   * replacement), or -1 if c was static or already alone.
   */
  splitChunk(c: number): BodyHandle {
    const id = this.islandOf[c];
    if (id < 0) { this.detachOne(Uint32Array.of(c), []); return -1; }
    const isl = this.islands[id];
    if (isl.chunks.length < 2) return -1;
    const rest = isl.chunks.filter((x) => x !== c), all = new Uint32Array(isl.chunks.length);
    all.set(rest); all[rest.length] = c;
    this.refracture(all, Uint32Array.of(0, rest.length, all.length), 2); // rest first: it reuses the island index
    return this.islands[id].body;
  }

  /**
   * Re-fractures islands split by breaks inside floating parts (PLAN §3): components chunks[offset[k]..offset[k+1]) from
   * integrity; an island whose chunks fall into 2+ components is replaced by one body per piece, each at the old body's
   * current pose and rigid-body velocity at its COM (so the render pose doesn't pop). @returns indices of new/reused islands.
   */
  refracture(chunks: Uint32Array, offset: Uint32Array, count: number): number[] {
    const pieces = new Map<number, Uint32Array[]>();
    for (let k = 0; k < count; k++) {
      const byIsland = new Map<number, number[]>();
      for (let q = offset[k]; q < offset[k + 1]; q++) {
        const id = this.islandOf[chunks[q]];
        if (id < 0) continue;
        let l = byIsland.get(id);
        if (!l) byIsland.set(id, (l = []));
        l.push(chunks[q]);
      }
      for (const [id, l] of byIsland) {
        let p = pieces.get(id);
        if (!p) pieces.set(id, (p = []));
        p.push(Uint32Array.from(l));
      }
    }
    const out: number[] = [];
    const st = new Float64Array(13), d = [0, 0, 0];
    for (const [id, ps] of pieces) {
      if (ps.length < 2) continue; // island still in one piece
      const old = this.islands[id];
      this.physics.readBodyStates([old.body], st);
      this.physics.removeBody(old.body);
      const q: [number, number, number, number] = [st[3], st[4], st[5], st[6]];
      ps.forEach((piece, i) => {
        // mass properties in the old body frame (positions = local t), so the new body keeps the old rotation
        for (const c of piece) for (let a = 0; a < 3; a++) this.work[3 * c + a] = old.local[7 * this.slotOf[c] + a];
        const mp = islandMassProperties(piece, 0, piece.length, this.mass, this.rotInertia, this.work);
        const local = new Float64Array(7 * piece.length);
        piece.forEach((c, k) => {
          const s = 7 * this.slotOf[c];
          for (let a = 0; a < 3; a++) local[7 * k + a] = old.local[s + a] - mp.com[a];
          for (let a = 0; a < 4; a++) local[7 * k + 3 + a] = old.local[s + 3 + a];
        });
        // x' = x_b + R_b com', v' = v + w x (x' - x_b)
        quatRotate(q, 0, mp.com, 0, d);
        this.createIsland(piece, local, mp, [st[0] + d[0], st[1] + d[1], st[2] + d[2]], q,
          [st[7] + st[11] * d[2] - st[12] * d[1], st[8] + st[12] * d[0] - st[10] * d[2], st[9] + st[10] * d[1] - st[11] * d[0]],
          [st[10], st[11], st[12]], i === 0 ? id : -1, out);
      });
    }
    return out;
  }

  /**
   * Re-attaches whole islands to the static structure at their current pose (joint-chain demotion, PLAN §4b): bodies
   * removed, one static collider per chunk at x_c = x_b + q_b t, q_c = q_b r, which becomes the chunk's static pose.
   * Remove the bodies' joints first. @param ids island indices.
   */
  attach(ids: ArrayLike<number>): void {
    const st = new Float64Array(13), x = [0, 0, 0], qc = [0, 0, 0, 1];
    for (let k = 0; k < ids.length; k++) {
      const isl = this.islands[ids[k]];
      if (isl.removed) continue;
      this.physics.readBodyStates([isl.body], st);
      this.physics.removeBody(isl.body);
      isl.chunks.forEach((c, s) => {
        quatRotate(st, 3, isl.local, 7 * s, x);
        quatMul(st, 3, isl.local, 7 * s + 3, qc);
        for (let a = 0; a < 3; a++) this.setPos[3 * c + a] = st[a] + x[a];
        for (let a = 0; a < 4; a++) this.setQ[4 * c + a] = qc[a];
        const h = chunkHull(this.model, c);
        h.translation = [this.setPos[3 * c], this.setPos[3 * c + 1], this.setPos[3 * c + 2]];
        h.rotation = [qc[0], qc[1], qc[2], qc[3]];
        this.staticCollider[c] = this.physics.createStaticChunk(this.staticBody, h, STRUCTURE_GROUPS);
        this.islandOf[c] = -1; this.slotOf[c] = -1;
        this.staticPoseDirty.push(c);
      });
      isl.removed = true; isl.colliders = []; isl.grace = 0;
    }
  }

  /** Sets static chunk c's visual pose (centre x (3,), quat q (4,)), e.g. a fake dent; colliders stay at rest (PLAN §4). */
  setStaticPose(c: number, x: ArrayLike<number>, q: ArrayLike<number>): void {
    for (let a = 0; a < 3; a++) this.setPos[3 * c + a] = x[a];
    for (let a = 0; a < 4; a++) this.setQ[4 * c + a] = q[a];
    this.staticPoseDirty.push(c);
  }

  /** Counts down grace timers by dt (s); expired islands switch to debris groups. */
  update(dt: number): void {
    for (const isl of this.islands) {
      if (isl.grace <= 0 || isl.removed) continue;
      isl.grace -= dt;
      if (isl.grace <= 0) for (const c of isl.colliders) this.physics.setCollisionGroups(c, isl.groups ?? DEBRIS_GROUPS);
    }
  }
}
