// Rapier implementation of PhysicsBackend. The only module that imports Rapier.
// Verified against @dimforge/rapier3d-compat 0.21 (see NOTES.md "Rapier API facts").

import RAPIER from '@dimforge/rapier3d-compat';
import type { BodyHandle, ColliderHandle, ImpactEvent, IslandChunk, IslandDesc, JointHandle, JointKind, PhysicsBackend } from './PhysicsBackend';

/** Loads the Rapier WASM once; must be awaited before creating a backend. */
export async function initRapier(): Promise<void> {
  await RAPIER.init();
}

export interface RapierBackendOptions {
  gravity?: [number, number, number];
  /** Contact-force event threshold (N); below this no impact event is emitted. */
  impactThreshold?: number;
  /** Rapier velocity solver iterations (default 4) and internal PGS iterations (default 1); higher = stiffer stacks. */
  solverIterations?: number;
  pgsIterations?: number;
}

export class RapierBackend implements PhysicsBackend {
  readonly world: RAPIER.World;
  private readonly events = new RAPIER.EventQueue(true);
  private readonly chunkOfCollider = new Map<number, number>();
  private readonly impactThreshold: number;

  /** @param opts gravity (m/s^2) and impact threshold (N). */
  constructor(opts: RapierBackendOptions = {}) {
    const g = opts.gravity ?? [0, -9.81, 0];
    this.world = new RAPIER.World({ x: g[0], y: g[1], z: g[2] });
    this.impactThreshold = opts.impactThreshold ?? 1000;
    if (opts.solverIterations) this.world.numSolverIterations = opts.solverIterations;
    if (opts.pgsIterations) this.world.numInternalPgsIterations = opts.pgsIterations;
  }

  step(dt: number): void {
    this.world.timestep = dt;
    this.world.step(this.events);
  }

  createStaticBody(): BodyHandle {
    return this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed()).handle;
  }

  /** Builds a convex collider desc from a chunk hull (indices supplied, so no quickhull). */
  private chunkDesc(chunk: IslandChunk, groups: number): RAPIER.ColliderDesc {
    const desc = RAPIER.ColliderDesc.convexMesh(chunk.points, chunk.indices);
    if (!desc) throw new Error(`convexMesh failed for chunk ${chunk.chunkId}`);
    if (chunk.rotation) desc.setRotation({ x: chunk.rotation[0], y: chunk.rotation[1], z: chunk.rotation[2], w: chunk.rotation[3] });
    return desc
      .setTranslation(chunk.translation[0], chunk.translation[1], chunk.translation[2])
      .setDensity(0) // mass comes from setAdditionalMassProperties: overlapping hulls would double count
      .setCollisionGroups(groups)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(this.impactThreshold);
  }

  createStaticChunk(body: BodyHandle, chunk: IslandChunk, groups: number): ColliderHandle {
    const c = this.world.createCollider(this.chunkDesc(chunk, groups), this.world.getRigidBody(body));
    this.chunkOfCollider.set(c.handle, chunk.chunkId);
    return c.handle;
  }

  removeCollider(c: ColliderHandle): void {
    const col = this.world.getCollider(c);
    if (!col) return;
    this.chunkOfCollider.delete(c);
    this.world.removeCollider(col, true);
  }

  createDynamicIsland(desc: IslandDesc): { body: BodyHandle; colliders: ColliderHandle[] } {
    const [qx, qy, qz, qw] = desc.rotation;
    const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(...desc.position)
      .setRotation({ x: qx, y: qy, z: qz, w: qw })
      .setLinvel(...desc.linvel)
      .setAngvel({ x: desc.angvel[0], y: desc.angvel[1], z: desc.angvel[2] })
      .setAdditionalMassProperties(
        desc.mass, { x: 0, y: 0, z: 0 },
        { x: desc.inertia[0], y: desc.inertia[1], z: desc.inertia[2] },
        { x: desc.inertiaFrame[0], y: desc.inertiaFrame[1], z: desc.inertiaFrame[2], w: desc.inertiaFrame[3] },
      );
    const body = this.world.createRigidBody(bodyDesc);
    const colliders = desc.chunks.map((ch) => {
      const c = this.world.createCollider(this.chunkDesc(ch, desc.groups), body);
      this.chunkOfCollider.set(c.handle, ch.chunkId);
      return c.handle;
    });
    return { body: body.handle, colliders };
  }

  removeBody(b: BodyHandle): void {
    const body = this.world.getRigidBody(b);
    if (!body) return;
    for (let i = 0; i < body.numColliders(); i++) this.chunkOfCollider.delete(body.collider(i).handle);
    this.world.removeRigidBody(body);
  }

  createGround(y: number, groups: number): BodyHandle {
    const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, y - 1, 0));
    this.world.createCollider(RAPIER.ColliderDesc.cuboid(500, 1, 500).setCollisionGroups(groups), body);
    return body.handle;
  }

  createSphere(radius: number, position: [number, number, number], linvel: [number, number, number], density: number, groups: number): BodyHandle {
    const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(...position).setLinvel(...linvel).setCcdEnabled(true));
    this.world.createCollider(
      RAPIER.ColliderDesc.ball(radius).setDensity(density).setCollisionGroups(groups)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS).setContactForceEventThreshold(this.impactThreshold),
      body,
    );
    return body.handle;
  }

  setChunkShape(c: ColliderHandle, points: Float32Array, indices: Uint32Array): void {
    const desc = RAPIER.ColliderDesc.convexMesh(points, indices);
    if (!desc) throw new Error('convexMesh failed in setChunkShape');
    this.world.getCollider(c).setShape(desc.shape);
  }

  setCollisionGroups(c: ColliderHandle, groups: number): void {
    this.world.getCollider(c).setCollisionGroups(groups);
  }

  createJoint(kind: JointKind, a: BodyHandle, b: BodyHandle, anchorA: [number, number, number], anchorB: [number, number, number]): JointHandle {
    const v = (p: [number, number, number]) => ({ x: p[0], y: p[1], z: p[2] });
    const id = { x: 0, y: 0, z: 0, w: 1 };
    const data = kind === 'fixed'
      ? RAPIER.JointData.fixed(v(anchorA), id, v(anchorB), id)
      : RAPIER.JointData.spherical(v(anchorA), v(anchorB));
    const j = this.world.createImpulseJoint(data, this.world.getRigidBody(a), this.world.getRigidBody(b), true);
    j.setContactsEnabled(false);
    return j.handle;
  }

  createSpringJoint(a: BodyHandle, b: BodyHandle, anchorA: [number, number, number], anchorB: [number, number, number],
    frameA: [number, number, number, number], frameB: [number, number, number, number],
    stiffness: [number, number, number], damping: [number, number, number]): JointHandle {
    const data = RAPIER.JointData.spherical({ x: anchorA[0], y: anchorA[1], z: anchorA[2] }, { x: anchorB[0], y: anchorB[1], z: anchorB[2] });
    const raw = this.world.createImpulseJoint(data, this.world.getRigidBody(a), this.world.getRigidBody(b), true);
    raw.setContactsEnabled(false);
    raw.setFrameX1({ x: frameA[0], y: frameA[1], z: frameA[2], w: frameA[3] });
    raw.setFrameX2({ x: frameB[0], y: frameB[1], z: frameB[2], w: frameB[3] });
    for (let i = 0; i < 3; i++) this.sphericalJoint(raw.handle).configureMotorModel(RAPIER.JointAxis.AngX + i, RAPIER.MotorModel.ForceBased);
    this.setSpringTarget(raw.handle, [0, 0, 0], stiffness, damping);
    return raw.handle;
  }

  /** createImpulseJoint types spherical joints as Generic (no motor API in 0.21): wrap the handle in the spherical class. */
  private sphericalJoint(j: JointHandle): RAPIER.SphericalImpulseJoint {
    return new RAPIER.SphericalImpulseJoint(this.world.impulseJoints.raw, this.world.bodies, j);
  }

  setSpringTarget(j: JointHandle, target: [number, number, number], stiffness: [number, number, number], damping: [number, number, number]): void {
    const joint = this.sphericalJoint(j);
    for (let i = 0; i < 3; i++) joint.configureMotorPosition(RAPIER.JointAxis.AngX + i, target[i], stiffness[i], damping[i]);
  }

  setBodyDamping(b: BodyHandle, linear: number, angular: number): void {
    const body = this.world.getRigidBody(b);
    body.setLinearDamping(linear);
    body.setAngularDamping(angular);
  }

  setJointAnchors(j: JointHandle, anchorA: [number, number, number], anchorB: [number, number, number]): void {
    const joint = this.world.getImpulseJoint(j);
    joint.setAnchor1({ x: anchorA[0], y: anchorA[1], z: anchorA[2] });
    joint.setAnchor2({ x: anchorB[0], y: anchorB[1], z: anchorB[2] });
  }

  sleepBodies(bodies: ArrayLike<BodyHandle>): void {
    for (let i = 0; i < bodies.length; i++) this.world.getRigidBody(bodies[i]).sleep();
  }

  removeJoint(j: JointHandle): void {
    const joint = this.world.getImpulseJoint(j);
    if (joint) this.world.removeImpulseJoint(joint, true);
  }

  readBodyPoses(bodies: ArrayLike<BodyHandle>, out: Float32Array): void {
    for (let i = 0; i < bodies.length; i++) {
      const body = this.world.getRigidBody(bodies[i]);
      const t = body.translation(), r = body.rotation();
      out[7 * i] = t.x; out[7 * i + 1] = t.y; out[7 * i + 2] = t.z;
      out[7 * i + 3] = r.x; out[7 * i + 4] = r.y; out[7 * i + 5] = r.z; out[7 * i + 6] = r.w;
    }
  }

  readBodyStates(bodies: ArrayLike<BodyHandle>, out: Float64Array): void {
    for (let i = 0; i < bodies.length; i++) {
      const body = this.world.getRigidBody(bodies[i]), o = 13 * i;
      const t = body.translation(), r = body.rotation(), v = body.linvel(), w = body.angvel();
      out[o] = t.x; out[o + 1] = t.y; out[o + 2] = t.z;
      out[o + 3] = r.x; out[o + 4] = r.y; out[o + 5] = r.z; out[o + 6] = r.w;
      out[o + 7] = v.x; out[o + 8] = v.y; out[o + 9] = v.z;
      out[o + 10] = w.x; out[o + 11] = w.y; out[o + 12] = w.z;
    }
  }

  applyWrenches(bodies: ArrayLike<BodyHandle>, wrench: Float64Array, dt: number): void {
    for (let i = 0; i < bodies.length; i++) {
      const body = this.world.getRigidBody(bodies[i]), o = 6 * i;
      if (!body.isDynamic()) continue;
      body.applyImpulse({ x: wrench[o] * dt, y: wrench[o + 1] * dt, z: wrench[o + 2] * dt }, true);
      body.applyTorqueImpulse({ x: wrench[o + 3] * dt, y: wrench[o + 4] * dt, z: wrench[o + 5] * dt }, true);
    }
  }

  drainImpactEvents(fn: (e: ImpactEvent) => void): void {
    this.events.drainContactForceEvents((e) => {
      const d = e.maxForceDirection();
      fn({
        chunkA: this.chunkOfCollider.get(e.collider1()) ?? -1,
        chunkB: this.chunkOfCollider.get(e.collider2()) ?? -1,
        force: e.totalForceMagnitude(),
        dir: [d.x, d.y, d.z],
      });
    });
  }

  /** Frees WASM resources. */
  dispose(): void {
    this.events.free();
    this.world.free();
  }
}
