// Island handoff (PLAN §3): detached components from tier 0 -> dynamic physics bodies.
// Removes the chunks' static colliders, creates one body per component at its COM with explicit mass/inertia,
// in the grace group (overlaps the structure without push-out) until `graceTime` passes, then normal debris groups.
// Talks to physics only through PhysicsBackend.

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { chunkMasses, chunkRotInertia } from '../integrity/loads';
import { islandMassProperties } from '../integrity/islandMass';
import { BodyHandle, ColliderHandle, GroupBit, PhysicsBackend, packGroups } from '../physics/PhysicsBackend';
import { chunkHull, islandDesc } from '../physics/fromModel';

export const STRUCTURE_GROUPS = packGroups(GroupBit.STRUCTURE, 0xffff & ~GroupBit.GRACE);
export const GRACE_GROUPS = packGroups(GroupBit.GRACE, GroupBit.DEBRIS);
export const DEBRIS_GROUPS = packGroups(GroupBit.DEBRIS, 0xffff);

export interface Island {
  body: BodyHandle;
  colliders: ColliderHandle[];
  chunks: Uint32Array;
  /** Rest-space COM = body origin at handoff; chunk body-local offset = chunkPos - com. */
  com: [number, number, number];
  /** Seconds left in the grace group (<= 0: normal debris). */
  grace: number;
}

export class IslandHandoff {
  readonly staticBody: BodyHandle;
  /** Static collider of each chunk, -1 once handed off: (n,). Float64: Rapier handles are non-integer doubles. */
  readonly staticCollider: Float64Array;
  /** Island index of each chunk, -1 = static structure: (n,). */
  readonly islandOf: Int32Array;
  readonly islands: Island[] = [];
  private readonly mass: Float64Array;
  private readonly rotInertia: Float64Array;

  /** Builds the static structure body with one collider per chunk. @param graceTime s in the grace group after detaching. */
  constructor(private readonly model: ChunkModel, table: MaterialTable, private readonly physics: PhysicsBackend,
    private readonly graceTime = 0.1) { // short: long enough to separate rest overlaps, short enough not to fall through ledges
    const n = model.chunkCount;
    this.mass = chunkMasses(model, table);
    this.rotInertia = chunkRotInertia(model, this.mass);
    this.staticBody = physics.createStaticBody();
    this.staticCollider = new Float64Array(n);
    this.islandOf = new Int32Array(n).fill(-1);
    for (let i = 0; i < n; i++) this.staticCollider[i] = physics.createStaticChunk(this.staticBody, chunkHull(model, i), STRUCTURE_GROUPS);
  }

  /**
   * Hands off components chunks[offset[k]..offset[k+1]) for k < count (Connectivity output layout).
   * Velocity 0: pieces leave the static structure (PLAN §3 step 3). @returns indices of the new islands.
   */
  detach(chunks: Uint32Array, offset: Uint32Array, count: number): number[] {
    const created: number[] = [];
    for (let k = 0; k < count; k++) {
      const from = offset[k], to = offset[k + 1];
      if (to === from || this.islandOf[chunks[from]] >= 0) continue; // already debris (island re-fracture not yet supported)
      for (let q = from; q < to; q++) {
        const c = chunks[q];
        if (this.staticCollider[c] !== -1) { this.physics.removeCollider(this.staticCollider[c]); this.staticCollider[c] = -1; }
      }
      const mp = islandMassProperties(chunks, from, to, this.mass, this.rotInertia, this.model.chunkPos);
      const { body, colliders } = this.physics.createDynamicIsland(islandDesc(this.model, chunks, from, to, mp, GRACE_GROUPS));
      const id = this.islands.length;
      const own = chunks.slice(from, to);
      for (const c of own) this.islandOf[c] = id;
      this.islands.push({ body, colliders, chunks: own, com: mp.com, grace: this.graceTime });
      created.push(id);
    }
    return created;
  }

  /** Counts down grace timers by dt (s); expired islands switch to debris groups. */
  update(dt: number): void {
    for (const isl of this.islands) {
      if (isl.grace <= 0) continue;
      isl.grace -= dt;
      if (isl.grace <= 0) for (const c of isl.colliders) this.physics.setCollisionGroups(c, DEBRIS_GROUPS);
    }
  }
}
