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
        { x: desc.inertia[0], y: desc.inertia[1], z: desc.inertia[2] }, { x: 0, y: 0, z: 0, w: 1 },
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
