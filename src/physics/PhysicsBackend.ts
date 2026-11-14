// Thin physics interface (PLAN top): everything outside physics/ talks to this, never to Rapier, so Havok is a swap.
// All arrays at this boundary are Float32 (the solver's Float64 stays on the other side).

export type BodyHandle = number;
export type ColliderHandle = number;
export type JointHandle = number;

/** One chunk collider on an island: hull in chunk-local coords, placed at `translation` relative to the body origin (= island COM). */
export interface IslandChunk {
  chunkId: number;
  points: Float32Array; // (3p,)
  indices: Uint32Array; // (3t,) hull triangles; supplied so no quickhull runs
  translation: [number, number, number];
}

export interface IslandDesc {
  /** World position of the body origin (island COM) and orientation (quaternion x,y,z,w). */
  position: [number, number, number];
  rotation: [number, number, number, number];
  chunks: IslandChunk[];
  /** Mass (kg), principal inertia (kg m^2) about the COM in the body frame (from rest cell volumes, PLAN §3). */
  mass: number;
  inertia: [number, number, number];
  linvel: [number, number, number];
  angvel: [number, number, number];
  /** Collision group bits (16 membership << 16 | 16 filter), e.g. the grace group for new islands. */
  groups: number;
}

export interface ImpactEvent {
  chunkA: number; // -1 if the collider is not a chunk
  chunkB: number;
  /** Total contact force magnitude (N) this step and its world direction (from the strongest contact). */
  force: number;
  dir: [number, number, number];
}

export type JointKind = 'fixed' | 'ball';

export interface PhysicsBackend {
  /** Advances the world by dt seconds. */
  step(dt: number): void;
  /** Fixed body holding the static structure; all static chunk colliders hang off it so joints can anchor to it. */
  createStaticBody(): BodyHandle;
  /** Adds a chunk collider to a fixed body at world `translation`. */
  createStaticChunk(body: BodyHandle, chunk: IslandChunk, groups: number): ColliderHandle;
  removeCollider(c: ColliderHandle): void;
  /** Creates one dynamic body with one convex collider per chunk and explicit mass properties. @returns body + collider handles (same order as desc.chunks). */
  createDynamicIsland(desc: IslandDesc): { body: BodyHandle; colliders: ColliderHandle[] };
  removeBody(b: BodyHandle): void;
  /** Infinite-ish static ground (thick box top at height y). */
  createGround(y: number, groups: number): BodyHandle;
  /** Dynamic sphere (projectile / wrecking ball), not a chunk. @param density kg/m^3 */
  createSphere(radius: number, position: [number, number, number], linvel: [number, number, number], density: number, groups: number): BodyHandle;
  /** Replaces a collider's hull (refit, PLAN §5); indices are supplied, no hull algorithm. */
  setChunkShape(c: ColliderHandle, points: Float32Array, indices: Uint32Array): void;
  setCollisionGroups(c: ColliderHandle, groups: number): void;
  /** Joint between two bodies at body-local anchors; contacts between them are disabled. */
  createJoint(kind: JointKind, a: BodyHandle, b: BodyHandle, anchorA: [number, number, number], anchorB: [number, number, number]): JointHandle;
  removeJoint(j: JointHandle): void;
  /** Writes pose (x, y, z, qx, qy, qz, qw) of each body into out (7 floats each). */
  readBodyPoses(bodies: ArrayLike<BodyHandle>, out: Float32Array): void;
  /** Calls fn for every contact-force event since the last drain. */
  drainImpactEvents(fn: (e: ImpactEvent) => void): void;
}

/** Packs Rapier-style interaction groups: membership bits (16) and filter bits (16). */
export function packGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16 | (filter & 0xffff)) >>> 0;
}

/** Collision group bits (PLAN 4b budget): bit 0 structure, bit 1 debris, bit 2 grace; the rest for part ids. */
export const GroupBit = { STRUCTURE: 1, DEBRIS: 2, GRACE: 4 } as const;
