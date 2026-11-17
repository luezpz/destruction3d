// Joint chains for `deform: 'joints'` parts (NOTES Design §4b): promotion of a static part to one dynamic body per chunk linked by
// spring joints (spherical + 3 implicit angular motors), per-step plasticity (motor target update) and joint fracture.
// Runs where Rapier runs (main thread). Handshake: a joint break is sent to integrity as a bond removal (same path as scripted
// cuts); promoted chunks are skipped by IslandHandoff.detach. Demotion (part at rest) re-attaches the bodies as static colliders
// at their deformed pose (IslandHandoff.attach); the integrity graph never saw the promotion, so it needs no message. Promotion is by impact or script until the external solver (M3b).

import { ChunkFlag, ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { buildBondAdjacency, BondAdjacency } from '../kernels/bondAdjacency';
import { BODY_STATE_STRIDE, computeJointSprings, JointViews, quatToMat } from '../kernels/jointSpring';
import { PlasticViews, updateJointPlasticity } from '../kernels/jointPlasticity';
import { chunkMasses, chunkRotInertia } from '../integrity/loads';
import { quatFromMatrix } from '../integrity/islandMass';
import { BodyHandle, JointHandle, PART_BIT0, PART_BITS, PhysicsBackend, packGroups } from '../physics/PhysicsBackend';
import { DEBRIS_GROUPS, IslandHandoff } from './IslandHandoff';

export interface JointChainOptions {
  /** Motor stiffness = raw section stiffness (GJ/L, EI/L) x this (spike: 1e-2 keeps motor compliance above the solver floor). */
  stiffnessScale?: number;
  /** Contact force (N) on a static `joints` chunk that promotes its part; Infinity disables impact promotion. */
  promoteForce?: number;
  /** Absolute body damping (1/s) while jointed: joint damping is relative and leaves whole-part modes ringing (never sleeps). */
  linearDamping?: number;
  angularDamping?: number;
  /** Rest test for sleeping a promoted part: every body below these speeds (m/s, rad/s) for `sleepTime` s. Rapier's own sleep takes seconds. */
  sleepLinear?: number;
  sleepAngular?: number;
  sleepTime?: number;
  /** Quiet time (s) after the last yield in a part before its current shape is baked as the new rest state. */
  bakeDelay?: number;
  /** Time (s) an anchored part must stay at rest (see sleep test) before demotion back to static colliders; Infinity disables. */
  demoteTime?: number;
}

/** What JointChains needs from the sim: static flags, bond liveness, the removal path and the island handoff. */
export interface ChainHost {
  readonly isStatic: Uint8Array;
  readonly alive: Uint8Array;
  readonly handoff: IslandHandoff;
  removeBonds(bonds: ArrayLike<number>): void;
}

/** One spring joint (setup-time record; the hot loop uses the typed views). */
interface JointRec {
  bond: number;
  handle: JointHandle;
  a: number; b: number; // body indices into `bodies`
  staticChunk: number; // chunk on the static side, -1 if both sides are promoted
  k: [number, number, number];
  c: [number, number, number];
}

export class JointChains {
  /** Body handles; index 0 = the static structure body. Float64 semantics: handles are non-integer doubles. */
  readonly bodies: BodyHandle[];
  /** Body index per chunk, -1 = not promoted: (n,). */
  readonly chunkBody: Int32Array;
  /** Joint index per bond, -1 = none: (m,). */
  readonly bondJoint: Int32Array;
  private joints: JointRec[] = [];
  private jv!: JointViews;
  private pv!: PlasticViews;
  private state = new Float64Array(0);
  private err = new Float64Array(0);
  private fLocal = new Float64Array(0);
  private wrench = new Float64Array(0);
  private phi = new Float64Array(0);
  private changed = new Uint8Array(0);
  private broken = new Uint8Array(0);
  /** Failure ratio per joint from the last step (debug). */
  ratio = new Float64Array(0);
  private readonly adj: BondAdjacency;
  private readonly rotInertia: Float64Array;
  private readonly scale: number;
  private readonly promoteForce: number;
  private pendingPromote: number[] = [];
  private readonly linDamp: number;
  private readonly angDamp: number;
  /** Alive joints per body index (body 0 = static, unused). A body losing its last joint drops the chain damping. */
  private readonly bodyJoints: number[] = [0];
  /** Promoted parts: body indices/handles, quiet time (s), alive joints to the static body (linear damping only while > 0). */
  private readonly parts: { bodies: number[]; handles: number[]; quiet: number; anchors: number; yielding: boolean; sinceYield: number }[] = [];
  /** Part index per body index (body 0 = static: -1). */
  private readonly bodyPart: number[] = [-1];
  /** Chunk per body index (body 0 = static: -1). */
  private readonly bodyChunk: number[] = [-1];
  /** Plastic strain (m,) and motor targets (3m,) per bond, kept across demotion: re-promotion resumes the same plastic state. */
  private readonly bondEpsP: Float64Array;
  private readonly bondTarget: Float64Array;
  private readonly demoteTime: number;
  /** Bodies demoted back to static since creation (debug counter). */
  demoted = 0;
  private readonly sleepLin: number;
  private readonly sleepAng: number;
  private readonly sleepTime: number;
  private readonly bakeDelay: number;
  /** Rest-state bakes since creation (debug counter). */
  bakes = 0;
  /** Joints removed by fracture since creation (debug counter). */
  brokenJoints = 0;

  constructor(private readonly model: ChunkModel, private readonly table: MaterialTable, private readonly physics: PhysicsBackend,
    private readonly host: ChainHost, o: JointChainOptions = {}) {
    this.scale = o.stiffnessScale ?? 1e-2;
    this.promoteForce = o.promoteForce ?? 2000;
    this.linDamp = o.linearDamping ?? 2; // settles a hit 3 cm steel plate in ~1.5 s (NOTES "Joint-chain perf")
    this.angDamp = o.angularDamping ?? 2;
    this.sleepLin = o.sleepLinear ?? 0.005;
    this.sleepAng = o.sleepAngular ?? 0.02;
    this.sleepTime = o.sleepTime ?? 0.25;
    this.bakeDelay = o.bakeDelay ?? 0.1;
    this.demoteTime = o.demoteTime ?? 1;
    this.bondEpsP = new Float64Array(model.bondCount);
    this.bondTarget = new Float64Array(3 * model.bondCount);
    this.adj = buildBondAdjacency(model.chunkCount, model.bondA, model.bondB, model.bondCount);
    this.rotInertia = chunkRotInertia(model, chunkMasses(model, table));
    this.bodies = [host.handoff.staticBody];
    this.chunkBody = new Int32Array(model.chunkCount).fill(-1);
    this.bondJoint = new Int32Array(model.bondCount).fill(-1);
    this.rebuildViews();
  }

  get jointCount(): number { return this.joints.length; }
  /** Alive joints (debug). */
  get aliveJoints(): number { let n = 0; for (let j = 0; j < this.joints.length; j++) n += this.pv.alive[j]; return n; }

  /** True if chunk c can be promoted: a static SKINNED, non-support chunk. */
  canPromote(c: number): boolean {
    const f = this.model.chunkFlags[c];
    return (f & ChunkFlag.SKINNED) !== 0 && (f & ChunkFlag.SUPPORT) === 0 && this.host.isStatic[c] === 1;
  }

  /**
   * Promotes the part containing `seed` (static SKINNED non-support chunks reachable through alive `joints` bonds): one body per
   * chunk, spring joints on its alive bonds, anchored to the static body where it touches the static structure. @returns chunks promoted.
   */
  promote(seed: number): number {
    if (!this.canPromote(seed)) return 0;
    const { model, host } = this;
    const part: number[] = [seed], inPart = new Set([seed]);
    for (let h = 0; h < part.length; h++) {
      const c = part[h];
      for (let i = this.adj.adjOffset[c]; i < this.adj.adjOffset[c + 1]; i++) {
        const b = this.adj.adjBond[i];
        if (!host.alive[b] || this.table.materials[model.bondMaterial[b]].deform !== 'joints') continue;
        const o = model.bondA[b] === c ? model.bondB[b] : model.bondA[b];
        if (!inPart.has(o) && this.canPromote(o)) { inPart.add(o); part.push(o); }
      }
    }
    // one single-chunk island per chunk (body origin = chunk centre, axes = world at rest)
    const chunks = Uint32Array.from(part), offset = Uint32Array.from({ length: part.length + 1 }, (_, k) => k);
    const ids = host.handoff.detach(chunks, offset, part.length);
    const rec = { bodies: [] as number[], handles: [] as number[], quiet: 0, anchors: 0, yielding: false, sinceYield: 0 };
    this.parts.push(rec);
    // part bit: no self-collision inside the part (bent chunks overlap diagonal neighbours and jitter against the joints)
    const bit = 1 << (PART_BIT0 + (this.parts.length - 1) % PART_BITS);
    const partGroups = packGroups(bit, 0xffff & ~bit); // membership = part bit only: a shared DEBRIS bit would re-enable self-contacts
    for (const id of ids) {
      host.handoff.islands[id].groups = partGroups;
      rec.bodies.push(this.bodies.length);
      rec.handles.push(host.handoff.islands[id].body);
      const isl = host.handoff.islands[id], c = isl.chunks[0];
      this.chunkBody[c] = this.bodies.length;
      this.bodies.push(isl.body);
      this.bodyJoints.push(0);
      this.bodyPart.push(this.parts.length - 1);
      this.bodyChunk.push(c);
      this.physics.setBodyDamping(isl.body, this.linDamp, this.angDamp);
      host.isStatic[c] = 0;
    }
    const firstNew = this.joints.length;
    for (const c of part) {
      for (let i = this.adj.adjOffset[c]; i < this.adj.adjOffset[c + 1]; i++) {
        const b = this.adj.adjBond[i];
        if (!host.alive[b] || this.bondJoint[b] >= 0) continue;
        const o = model.bondA[b] === c ? model.bondB[b] : model.bondA[b];
        if (this.chunkBody[o] >= 0 || host.isStatic[o]) this.addJoint(b);
      }
    }
    // a previously demoted part restarts with its stored targets (it rested in equilibrium with
    // them, a bake would drop the load and re-yield it) and anchors re-closed at the frozen pose
    this.rebuildViews();
    this.physics.readBodyStates(this.bodies, this.state);
    for (let j = firstNew; j < this.joints.length; j++) {
      const r = this.joints[j], t = this.pv.target;
      if (t[3 * j] || t[3 * j + 1] || t[3 * j + 2]) this.physics.setSpringTarget(r.handle, [t[3 * j], t[3 * j + 1], t[3 * j + 2]], r.k, r.c);
      this.resetAnchor(j);
    }
    return part.length;
  }

  /** Creates the spring joint of bond b between its sides' bodies (static side = body 0). */
  private addJoint(b: number): void {
    const { model } = this;
    const ca = model.bondA[b], cb = model.bondB[b];
    const ia = this.chunkBody[ca] >= 0 ? this.chunkBody[ca] : 0, ib = this.chunkBody[cb] >= 0 ? this.chunkBody[cb] : 0;
    const origin = (c: number, i: number): number[] => (i === 0 ? [0, 0, 0] : [model.chunkPos[3 * c], model.chunkPos[3 * c + 1], model.chunkPos[3 * c + 2]]);
    const oa = origin(ca, ia), ob = origin(cb, ib), cen = model.bondCentroid.subarray(3 * b, 3 * b + 3);
    const anchorA: [number, number, number] = [cen[0] - oa[0], cen[1] - oa[1], cen[2] - oa[2]];
    const anchorB: [number, number, number] = [cen[0] - ob[0], cen[1] - ob[1], cen[2] - ob[2]];
    const n = model.bondNormal, t1 = model.bondT1, t2 = model.bondT2, b3 = 3 * b;
    // R = [n t1 t2] as columns, row-major
    const R = [n[b3], t1[b3], t2[b3], n[b3 + 1], t1[b3 + 1], t2[b3 + 1], n[b3 + 2], t1[b3 + 2], t2[b3 + 2]];
    const frame = quatFromMatrix(R);
    const mat = this.table.materials[model.bondMaterial[b]];
    const E = mat.E * 1e9, G = E / (2 * (1 + mat.nu));
    const L = Math.hypot(model.chunkPos[3 * ca] - model.chunkPos[3 * cb], model.chunkPos[3 * ca + 1] - model.chunkPos[3 * cb + 1], model.chunkPos[3 * ca + 2] - model.chunkPos[3 * cb + 2]);
    const k: [number, number, number] = [G * model.bondJ[b] / L * this.scale, E * model.bondI1[b] / L * this.scale, E * model.bondI2[b] / L * this.scale];
    // zeta ~0.5 against the lighter dynamic side's own rotational inertia
    const Iown = Math.min(ia ? this.rotInertia[ca] : Infinity, ib ? this.rotInertia[cb] : Infinity);
    const c = k.map((kk) => Math.sqrt(kk * Iown)) as [number, number, number];
    const handle = this.physics.createSpringJoint(this.bodies[ia], this.bodies[ib], anchorA, anchorB, frame, frame, k, c);
    this.bodyJoints[ia]++; this.bodyJoints[ib]++;
    if (ia === 0 || ib === 0) this.parts[this.bodyPart[ia || ib]].anchors++;
    this.bondJoint[b] = this.joints.length;
    this.joints.push({ bond: b, handle, a: ia, b: ib, staticChunk: ia === 0 ? ca : ib === 0 ? cb : -1, k, c });
  }

  /** Rebuilds the typed views from the joint records, keeping targets/strain/alive of existing joints (setup-time, allocates). */
  private rebuildViews(): void {
    const { model } = this, m = this.joints.length, old = this.pv, oldJ = this.jv;
    const jv: JointViews = {
      jointA: new Uint32Array(m), jointB: new Uint32Array(m), anchorA: new Float64Array(3 * m), anchorB: new Float64Array(3 * m),
      frame: new Float64Array(9 * m), restQ: new Float64Array(4 * m), restD: new Float64Array(3 * m),
      k: new Float64Array(6 * m), c: new Float64Array(6 * m), alive: new Uint8Array(m),
    };
    let corners = 0;
    for (const j of this.joints) corners += model.bondCornerOffset[j.bond + 1] - model.bondCornerOffset[j.bond];
    const pv: PlasticViews = {
      cornerOffset: new Uint32Array(m + 1), cornerYZ: new Float64Array(2 * corners),
      I1: new Float64Array(m), I2: new Float64Array(m), J: new Float64Array(m), length: new Float64Array(m), rMax: new Float64Array(m),
      k: new Float64Array(3 * m), sigmaY: new Float64Array(m), H: new Float64Array(m), E: new Float64Array(m), sigmaT: new Float64Array(m),
      epsPlasticMax: new Float64Array(m), epsElasticMax: new Float64Array(m),
      target: new Float64Array(3 * m), epsP: new Float64Array(m), alive: new Uint8Array(m),
    };
    let cc = 0;
    for (let j = 0; j < m; j++) {
      const r = this.joints[j], b = r.bond, b3 = 3 * b;
      const ca = model.bondA[b], cb = model.bondB[b];
      jv.jointA[j] = r.a; jv.jointB[j] = r.b;
      for (let i = 0; i < 3; i++) {
        const cen = model.bondCentroid[b3 + i];
        jv.anchorA[3 * j + i] = cen - (r.a ? model.chunkPos[3 * ca + i] : 0);
        jv.anchorB[3 * j + i] = cen - (r.b ? model.chunkPos[3 * cb + i] : 0);
        // frame row i = [n_i t1_i t2_i] (bodies have world axes at rest, so B in a's frame = world bond frame)
        jv.frame[9 * j + 3 * i] = model.bondNormal[b3 + i];
        jv.frame[9 * j + 3 * i + 1] = model.bondT1[b3 + i];
        jv.frame[9 * j + 3 * i + 2] = model.bondT2[b3 + i];
        pv.k[3 * j + i] = r.k[i];
      }
      jv.restQ[4 * j + 3] = 1;
      const mat = this.table.materials[model.bondMaterial[b]];
      pv.I1[j] = model.bondI1[b]; pv.I2[j] = model.bondI2[b]; pv.J[j] = model.bondJ[b];
      pv.length[j] = Math.hypot(model.chunkPos[3 * ca] - model.chunkPos[3 * cb], model.chunkPos[3 * ca + 1] - model.chunkPos[3 * cb + 1], model.chunkPos[3 * ca + 2] - model.chunkPos[3 * cb + 2]);
      pv.sigmaY[j] = mat.sigmaY * 1e6; pv.H[j] = mat.H * 1e9; pv.E[j] = mat.E * 1e9; pv.sigmaT[j] = mat.sigmaT * 1e6;
      pv.epsPlasticMax[j] = mat.epsPlasticMax; pv.epsElasticMax[j] = mat.epsElasticMax;
      pv.cornerOffset[j] = cc;
      let rMax = 0;
      for (let q = model.bondCornerOffset[b]; q < model.bondCornerOffset[b + 1]; q++) {
        const dx = model.bondCorners[3 * q] - model.bondCentroid[b3], dy = model.bondCorners[3 * q + 1] - model.bondCentroid[b3 + 1], dz = model.bondCorners[3 * q + 2] - model.bondCentroid[b3 + 2];
        const y = dx * model.bondT1[b3] + dy * model.bondT1[b3 + 1] + dz * model.bondT1[b3 + 2];
        const z = dx * model.bondT2[b3] + dy * model.bondT2[b3 + 1] + dz * model.bondT2[b3 + 2];
        pv.cornerYZ[2 * cc] = y; pv.cornerYZ[2 * cc + 1] = z; cc++;
        rMax = Math.max(rMax, Math.hypot(y, z));
      }
      pv.rMax[j] = rMax;
      pv.epsP[j] = this.bondEpsP[b];
      for (let i = 0; i < 3; i++) pv.target[3 * j + i] = this.bondTarget[3 * b + i];
      const keep = old && j < old.alive.length;
      pv.alive[j] = jv.alive[j] = keep ? old.alive[j] : 1;
      if (keep) {
        pv.epsP[j] = old.epsP[j];
        for (let i = 0; i < 3; i++) {
          pv.target[3 * j + i] = old.target[3 * j + i];
          jv.anchorA[3 * j + i] = oldJ.anchorA[3 * j + i]; jv.anchorB[3 * j + i] = oldJ.anchorB[3 * j + i]; // keep bakes
        }
      }
    }
    pv.cornerOffset[m] = cc;
    this.jv = jv; this.pv = pv;
    const nb = this.bodies.length;
    this.state = new Float64Array(BODY_STATE_STRIDE * nb); this.wrench = new Float64Array(6 * nb);
    this.err = new Float64Array(6 * m); this.fLocal = new Float64Array(6 * m); this.phi = new Float64Array(3 * m);
    this.changed = new Uint8Array(m); this.broken = new Uint8Array(m); this.ratio = new Float64Array(m);
  }

  /**
   * Queues impact promotion from a contact force event (call per event). @param chunk chunk hit (-1 = none);
   * @param force contact force (N). @returns true if the hit promotes (or already queued) a static part.
   */
  onImpact(chunk: number, force: number): boolean {
    if (chunk < 0 || force < this.promoteForce || !this.canPromote(chunk)) return false;
    if (!this.pendingPromote.includes(chunk)) this.pendingPromote.push(chunk);
    return true;
  }

  /** Per physics step, after `physics.step`: promotions, joint errors, plasticity (motor targets), fracture -> bond removals. */
  step(dt: number): void {
    for (const c of this.pendingPromote) this.promote(c);
    this.pendingPromote.length = 0;
    const m = this.joints.length;
    if (!m) return;
    const { pv, host } = this;
    // a static anchor that left the structure (detached as debris) can no longer hold its joint
    // and a bond cut elsewhere (scripted / integrity) drops its joint
    for (let j = 0; j < m; j++) {
      if (!pv.alive[j]) continue;
      const sc = this.joints[j].staticChunk;
      if (!host.alive[this.joints[j].bond]) this.breakJoint(j, false);
      else if (sc >= 0 && !host.isStatic[sc]) this.breakJoint(j);
    }
    this.physics.readBodyStates(this.bodies, this.state);
    this.sleepSettledParts(dt);
    this.computePhi();
    updateJointPlasticity(pv, this.phi, this.changed, this.broken, this.ratio, m);
    for (let j = 0; j < m; j++) {
      if (this.broken[j]) { this.breakJoint(j); continue; }
      if (this.changed[j]) {
        const r = this.joints[j];
        this.physics.setSpringTarget(r.handle, [pv.target[3 * j], pv.target[3 * j + 1], pv.target[3 * j + 2]], r.k, r.c);
        const p = this.parts[this.bodyPart[r.a || r.b]];
        p.yielding = true; p.sinceYield = 0;
      }
    }
    this.bakeSettledYields(dt);
    this.demoteRestingParts();
  }

  /** Joint rotation errors phi (3m,) from the body states read this step (k = c = 0 in the views: errors only). */
  private computePhi(): void {
    const m = this.joints.length;
    computeJointSprings(this.jv, this.state, this.err, this.fLocal, this.wrench, m);
    for (let j = 0; j < m; j++) for (let i = 0; i < 3; i++) this.phi[3 * j + i] = this.err[6 * j + 3 + i];
  }

  /**
   * Demotion (NOTES Design §4b): an anchored part at rest for demoteTime, not yielding, goes back to the static structure. The
   * bodies reachable from the static body through alive joints are re-attached as static colliders at their current
   * (deformed) pose; their joints are dropped without bond removal (the bonds stay intact), eps_p is kept per bond.
   */
  private demoteRestingParts(): void {
    for (let pi = 0; pi < this.parts.length; pi++) {
      const p = this.parts[pi];
      if (p.anchors === 0 || p.yielding || p.quiet < this.demoteTime || !p.bodies.length) continue;
      // bodies reachable from the static body (index 0) through alive joints of this part
      const reach = new Set<number>([0]);
      for (let grew = true; grew;) {
        grew = false;
        for (let j = 0; j < this.joints.length; j++) {
          const r = this.joints[j];
          if (!this.pv.alive[j] || this.bodyPart[r.a || r.b] !== pi || reach.has(r.a) === reach.has(r.b)) continue;
          reach.add(r.a); reach.add(r.b); grew = true;
        }
      }
      reach.delete(0);
      for (let j = 0; j < this.joints.length; j++) {
        const r = this.joints[j];
        if (!this.pv.alive[j] || !(reach.has(r.a) || reach.has(r.b))) continue;
        this.pv.alive[j] = this.jv.alive[j] = 0;
        this.physics.removeJoint(r.handle);
        this.bondEpsP[r.bond] = this.pv.epsP[j];
        for (let i = 0; i < 3; i++) this.bondTarget[3 * r.bond + i] = this.pv.target[3 * j + i];
        this.bondJoint[r.bond] = -1;
        if (r.a === 0 || r.b === 0) p.anchors--;
      }
      const ids: number[] = [];
      for (const i of reach) {
        const c = this.bodyChunk[i];
        ids.push(this.host.handoff.islandOf[c]);
        this.host.isStatic[c] = 1;
        this.chunkBody[c] = -1;
        this.bodyJoints[i] = 0;
        this.bodies[i] = this.host.handoff.staticBody; // keeps body indices stable; dead joints still point here
      }
      this.host.handoff.attach(ids);
      const keep = p.bodies.map((i, k) => (reach.has(i) ? -1 : k)).filter((k) => k >= 0);
      p.handles = keep.map((k) => p.handles[k]); p.bodies = keep.map((k) => p.bodies[k]);
      p.quiet = 0;
      this.demoted += reach.size;
    }
  }

  /**
   * Rest-state bake (NOTES Design §4b): in 2D parts (grids with loops) independently updated plastic targets can't all be met with
   * locked anchors, so the part stays strained and the solver jitters forever (tests/chainPerf). Once a part has stopped
   * yielding for bakeDelay, its current shape becomes the stress-free rest: target = phi, anchors moved together.
   * Cost: residual elastic stress at bake time becomes permanent set (at most ~one elastic sag per yield episode).
   */
  private bakeSettledYields(dt: number): void {
    for (let pi = 0; pi < this.parts.length; pi++) {
      const p = this.parts[pi];
      if (!p.yielding) continue;
      p.sinceYield += dt;
      if (p.sinceYield < this.bakeDelay) continue;
      p.yielding = false;
      if (this.isTree(pi)) continue; // no loops = no incompatibility; baking would drop a sustained load and ratchet
      this.bakePart(pi);
      this.bakes++;
    }
  }

  /** True if part pi's alive joints form no loop (static body = one node): E = alive bodies (connected). */
  private isTree(pi: number): boolean {
    let e = 0, v = 0;
    for (let j = 0; j < this.joints.length; j++) if (this.pv.alive[j] && this.bodyPart[this.joints[j].a || this.joints[j].b] === pi) e++;
    for (const i of this.parts[pi].bodies) if (this.bodyJoints[i] > 0) v++;
    return e <= v;
  }

  /** Makes part pi's current shape its stress-free rest: target = phi, dynamic anchors moved onto the other side's. */
  private bakePart(pi: number): void {
    for (let j = 0; j < this.joints.length; j++) {
      const r = this.joints[j];
      if (!this.pv.alive[j] || this.bodyPart[r.a || r.b] !== pi) continue;
      for (let i = 0; i < 3; i++) this.pv.target[3 * j + i] = this.phi[3 * j + i];
      this.physics.setSpringTarget(r.handle, [this.phi[3 * j], this.phi[3 * j + 1], this.phi[3 * j + 2]], r.k, r.c);
      this.resetAnchor(j);
    }
  }

  /** Moves joint j's dynamic-side anchor onto the other side's current anchor: anchor_d = R_d^T (x_o + R_o anchor_o - x_d). */
  private resetAnchor(j: number): void {
    const r = this.joints[j], jv = this.jv, s = this.state;
    const moveB = r.b !== 0; // move the dynamic side's anchor (static body never moves)
    const d = moveB ? r.b : r.a, o = moveB ? r.a : r.b;
    const ancO = moveB ? jv.anchorA : jv.anchorB, ancD = moveB ? jv.anchorB : jv.anchorA, j3 = 3 * j;
    const so = BODY_STATE_STRIDE * o, sd = BODY_STATE_STRIDE * d;
    quatToMat(s, so + 3, this.Ro); quatToMat(s, sd + 3, this.Rd);
    const Ro = this.Ro, Rd = this.Rd;
    const wx = s[so] + Ro[0] * ancO[j3] + Ro[1] * ancO[j3 + 1] + Ro[2] * ancO[j3 + 2] - s[sd];
    const wy = s[so + 1] + Ro[3] * ancO[j3] + Ro[4] * ancO[j3 + 1] + Ro[5] * ancO[j3 + 2] - s[sd + 1];
    const wz = s[so + 2] + Ro[6] * ancO[j3] + Ro[7] * ancO[j3 + 1] + Ro[8] * ancO[j3 + 2] - s[sd + 2];
    // R_d^T w (columns of R_d)
    ancD[j3] = Rd[0] * wx + Rd[3] * wy + Rd[6] * wz;
    ancD[j3 + 1] = Rd[1] * wx + Rd[4] * wy + Rd[7] * wz;
    ancD[j3 + 2] = Rd[2] * wx + Rd[5] * wy + Rd[8] * wz;
    this.physics.setJointAnchors(r.handle,
      [jv.anchorA[j3], jv.anchorA[j3 + 1], jv.anchorA[j3 + 2]], [jv.anchorB[j3], jv.anchorB[j3 + 1], jv.anchorB[j3 + 2]]);
  }

  private readonly Ro = new Float64Array(9);
  private readonly Rd = new Float64Array(9);

  /** A body that lost its last joint is plain debris: no damping, collides with its old part again. */
  private freeBody(i: number): void {
    this.physics.setBodyDamping(this.bodies[i], 0, 0);
    const isl = this.host.handoff.islands.find((x) => x.body === this.bodies[i]);
    if (!isl) return;
    isl.groups = undefined;
    if (isl.grace <= 0) for (const c of isl.colliders) this.physics.setCollisionGroups(c, DEBRIS_GROUPS);
  }

  /** Sleeps parts whose bodies all stayed below the rest speeds for sleepTime (uses the states read this step). */
  private sleepSettledParts(dt: number): void {
    const s = this.state, l2 = this.sleepLin ** 2, a2 = this.sleepAng ** 2;
    for (const p of this.parts) {
      let quiet = true;
      for (const i of p.bodies) {
        const o = BODY_STATE_STRIDE * i;
        if (s[o + 7] ** 2 + s[o + 8] ** 2 + s[o + 9] ** 2 > l2 || s[o + 10] ** 2 + s[o + 11] ** 2 + s[o + 12] ** 2 > a2) { quiet = false; break; }
      }
      p.quiet = quiet ? p.quiet + dt : 0;
      if (p.quiet >= this.sleepTime && p.quiet - dt < this.sleepTime) this.physics.sleepBodies(p.handles); // once per quiet spell
    }
  }

  /** Removes joint j; `send` = also send its bond to integrity as a removal (false when integrity already broke it). */
  private breakJoint(j: number, send = true): void {
    this.pv.alive[j] = 0; this.jv.alive[j] = 0;
    this.physics.removeJoint(this.joints[j].handle);
    for (const i of [this.joints[j].a, this.joints[j].b]) {
      if (i > 0 && --this.bodyJoints[i] === 0) this.freeBody(i);
    }
    const r = this.joints[j];
    if (r.a === 0 || r.b === 0) {
      const p = this.parts[this.bodyPart[r.a || r.b]];
      // last anchor gone: the part falls freely, keep only angular damping (linear damping would slow the fall)
      if (--p.anchors === 0) for (const i of p.bodies) if (this.bodyJoints[i] > 0) this.physics.setBodyDamping(this.bodies[i], 0, this.angDamp);
    }
    if (send) { this.host.removeBonds([this.joints[j].bond]); this.brokenJoints++; }
  }

  /** Plastic target (3,) and strain of the joint on bond b (tests/debug); null if none. */
  jointState(b: number): { target: number[]; epsP: number; alive: boolean; gap: number } | null {
    const j = this.bondJoint[b];
    if (j < 0) return null;
    return { target: Array.from(this.pv.target.subarray(3 * j, 3 * j + 3)), epsP: this.pv.epsP[j], alive: this.pv.alive[j] === 1,
      gap: Math.hypot(this.err[6 * j], this.err[6 * j + 1], this.err[6 * j + 2]) }; // anchor gap (m): solver residual of the translation lock
  }
}
