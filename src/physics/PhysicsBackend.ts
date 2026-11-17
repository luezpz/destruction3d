// Thin physics interface (NOTES Design intro): everything outside physics/ talks to this, never to Rapier, so Havok is a swap.
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
  /** Collider rotation (quat x,y,z,w) relative to the body; default identity. Set for deformed (demoted) chunks. */
  rotation?: [number, number, number, number];
}

export interface IslandDesc {
  /** World position of the body origin (island COM) and orientation (quaternion x,y,z,w). */
  position: [number, number, number];
  rotation: [number, number, number, number];
  chunks: IslandChunk[];
  /** Mass (kg), principal inertia (kg m^2) about the COM in the body frame (from rest cell volumes, NOTES Design §3). */
  mass: number;
  inertia: [number, number, number];
  /** Rotation (quaternion x,y,z,w) of the principal inertia axes in the body frame. */
  inertiaFrame: [number, number, number, number];
  linvel: [number, number, number];
  angvel: [number, number, number];
  /** Collision group bits (16 membership << 16 | 16 filter), e.g. the grace group for new islands. */
  groups: number;
}

export interface ImpactEvent {
  chunkA: number; // -1 if the collider is not a chunk
  chunkB: number;
  /**
   * Total contact force magnitude (N) this step and its world direction (from the strongest contact).
   * `dir` points from A to B: the force on B is +dir * force, on A -dir * force (verified in tests/rapier.test.ts).
   */
  force: number;
  dir: [number, number, number];
  /** Total contact force on B (N, world, = -force on A); impulse this step = forceVec * dt. */
  forceVec: [number, number, number];
  /** Bodies owning collider 1 / 2. */
  bodyA: BodyHandle;
  bodyB: BodyHandle;
  /** Linear (CoM) velocity of each body before this step (m/s, world); zero for fixed/sleeping bodies. */
  vPreA: [number, number, number];
  vPreB: [number, number, number];
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
  /** Replaces a collider's hull (refit, NOTES Design §5); indices are supplied, no hull algorithm. */
  setChunkShape(c: ColliderHandle, points: Float32Array, indices: Uint32Array): void;
  setCollisionGroups(c: ColliderHandle, groups: number): void;
  /** Joint between two bodies at body-local anchors; contacts between them are disabled. */
  createJoint(kind: JointKind, a: BodyHandle, b: BodyHandle, anchorA: [number, number, number], anchorB: [number, number, number]): JointHandle;
  /**
   * Bendable bond (NOTES Design §4b): translation locked at the anchors, rotation sprung by implicit motors per joint-frame axis
   * (x = bond normal: torsion GJ/L; y, z: bending EI/L) toward `target` angles. frameA/frameB: joint frame in each body (quat x,y,z,w).
   * @param stiffness (N m/rad), damping (N m s/rad) per axis.
   */
  createSpringJoint(a: BodyHandle, b: BodyHandle, anchorA: [number, number, number], anchorB: [number, number, number],
    frameA: [number, number, number, number], frameB: [number, number, number, number],
    stiffness: [number, number, number], damping: [number, number, number]): JointHandle;
  /** Moves the motor rest angles of a spring joint (plastic set); stiffness/damping as at creation. */
  setSpringTarget(j: JointHandle, target: [number, number, number], stiffness: [number, number, number], damping: [number, number, number]): void;
  removeJoint(j: JointHandle): void;
  /** Absolute velocity damping (1/s): damps whole-body modes joint-relative damping leaves (joint chains, NOTES Design §4b). */
  setBodyDamping(b: BodyHandle, linear: number, angular: number): void;
  /** Puts bodies to sleep now (contacts/impulses wake them); for parts our own rest test says are settled. */
  sleepBodies(bodies: ArrayLike<BodyHandle>): void;
  /** Moves a joint's body-local anchors (translational plastic set). */
  setJointAnchors(j: JointHandle, anchorA: [number, number, number], anchorB: [number, number, number]): void;
  /** Writes pose (x, y, z, qx, qy, qz, qw) of each body into out (7 floats each). */
  readBodyPoses(bodies: ArrayLike<BodyHandle>, out: Float32Array): void;
  /** Writes x(3) q(4) linvel(3) angvel(3) of each body (13 doubles each, world frame) for joint springs (NOTES Design §4b). */
  readBodyStates(bodies: ArrayLike<BodyHandle>, out: Float64Array): void;
  /** Applies one-step impulses from world wrenches (force(3), torque(3) per body) times dt; wakes the bodies. */
  applyWrenches(bodies: ArrayLike<BodyHandle>, wrench: Float64Array, dt: number): void;
  /** Body mass (kg); Infinity for fixed/kinematic bodies (they take no impulse). */
  bodyMass(b: BodyHandle): number;
  /** Applies a world impulse (N s) at the centre of mass and wakes the body (no-op for non-dynamic bodies). */
  applyImpulse(b: BodyHandle, impulse: [number, number, number]): void;
  /** Calls fn for every contact-force event since the last drain. */
  drainImpactEvents(fn: (e: ImpactEvent) => void): void;
}

/** Packs Rapier-style interaction groups: membership bits (16) and filter bits (16). */
export function packGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16 | (filter & 0xffff)) >>> 0;
}

/** Collision group bits (NOTES Design §4b budget): bit 0 structure, bit 1 debris, bit 2 grace; the rest for part ids. */
export const GroupBit = { STRUCTURE: 1, DEBRIS: 2, GRACE: 4 } as const;
/** Bits 3..15 tag joint-chain parts (`partId mod PART_BITS`); parts sharing a bit don't collide (rare, NOTES Design §4b). */
export const PART_BIT0 = 3;
export const PART_BITS = 13;
