// Universal collision fracture: every contact pair (projectile, debris, falling islands, ground, static structure) can
// break the chunks it touches; the pair then keeps closing (minus the fracture energy) instead of bouncing.
//
// Per contact-force event between bodies A, B (Rapier has already solved the step):
//   n = F_tot / |F_tot|          (3,) unit contact normal, A -> B (force on B is +n)
//   v_n^- = (v_A^- - v_B^-) . n   CoM closing speed before the step, from the backend's pre-step velocities (m/s)
//   mu = 1 / (1/m_A + 1/m_B)      reduced mass (kg; fixed bodies 1/m = 0)
// Everything is at the centre of mass (velocities, mass, hand-back impulse): an off-centre hand-back would add spin the
// CoM mass doesn't account for (effective mass at a lever is smaller), which created energy. Rapier's own angular
// response to the contact is left as is.
//   E_rel = 1/2 mu (v_n^-)^2      energy available to fracture (J); dt-independent, unlike the event force
// Touching chunks with intact bonds break cheapest-first while E_rel - spent >= E_break(c),
//   E_break(c) = s_E V_c (1/2 sigmaT eps_el + sigmaY eps_pl)   (J; V m^3, MPa -> Pa: strain energy at failure).
// One body pair touching several chunks in a step (one event per collider pair) shares one energy budget.
// After a break the pair should still close at v_n' = v_n^- sqrt(1 - spent / E_rel). Rapier left it at v_n^+ (measured
// when handing back, after earlier hand-backs this step), so we hand back dJ = clamp(mu (v_n' - v_n^+), 0, |P|):
// +dJ n on A, -dJ n on B (raises closing speed), never more than the impulse Rapier applied (P = F_tot dt). No energy
// is created, and penetration-recovery impulses can't feed back into bigger "impacts" (pre-step velocities lack them).
// The caller frees the cut chunks right away (IslandHandoff.splitChunk), so nothing ever passes through a chunk that
// is still solid, and bodies never sink into the ground waiting for an integrity reply.

import { buildBondAdjacency, BondAdjacency } from '../kernels/bondAdjacency';
import { ChunkFlag, ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';
import { ImpactEvent, PhysicsBackend } from '../physics/PhysicsBackend';

export interface CollisionBreakOptions {
  /** s_E: multiplier on the fracture energy (Infinity disables collision breaking). */
  energyScale?: number;
  /** Closing speeds below this (m/s) never break: resting contacts under heavy islands. */
  minSpeed?: number;
}

/** What one event decided (chunks to free now, then `handBack`). */
export interface BreakDecision {
  /** Chunks to free (1..2). */
  chunks: number[];
  /** Unit contact normal A -> B (3,). */
  n: [number, number, number];
  /** Closing speed the pair should keep (m/s) and the most impulse to hand back (N s, = Rapier's |P|). */
  vTarget: number;
  maxJ: number;
}

export class CollisionBreaker {
  /** E_break per chunk (J): (n,). Infinity = never breaks by collision (supports, joint-chain parts). */
  readonly breakEnergy: Float64Array;
  /** Per chunk (n,): 1 once cut by a collision (bonds removal may still be in flight). */
  readonly cut: Uint8Array;
  /** Debug counters: chunks broken by collisions, total fracture energy (J). */
  broken = 0;
  energy = 0;
  minSpeed: number;
  private readonly adj: BondAdjacency;
  private readonly st = new Float64Array(13);
  private readonly imp: [number, number, number] = [0, 0, 0];
  /** Fracture energy already spent per body pair this step (J), keyed "lo|hi" handle. */
  private readonly pairSpent = new Map<string, number>();

  /** @param model chunks/bonds; @param table materials; @param physics masses, post-step states; @param opts tuning. Setup-time, allocates. */
  constructor(model: ChunkModel, table: MaterialTable, private readonly physics: PhysicsBackend, opts: CollisionBreakOptions = {}) {
    const n = model.chunkCount, sE = opts.energyScale ?? 1;
    this.minSpeed = opts.minSpeed ?? 0.5;
    this.adj = buildBondAdjacency(n, model.bondA, model.bondB, model.bondCount);
    this.breakEnergy = new Float64Array(n);
    this.cut = new Uint8Array(n);
    for (let c = 0; c < n; c++) {
      // supports stay fixed; skinned (joint-chain) chunks fracture through their joints, not here
      if (model.chunkFlags[c] & (ChunkFlag.SUPPORT | ChunkFlag.SKINNED)) { this.breakEnergy[c] = Infinity; continue; }
      const m = table.materials[model.chunkMaterial[c]];
      const u = (0.5 * m.sigmaT * m.epsElasticMax + m.sigmaY * m.epsPlasticMax) * 1e6; // J/m^3
      this.breakEnergy[c] = sE * u * model.chunkVolume[c];
    }
  }

  /** Call before each step's event drain: fracture energy is shared per body pair within a step. */
  beginStep(): void { this.pairSpent.clear(); }

  /**
   * Decides one contact event (see header). @param e event; @param alive (m,) intact bonds; @param bonds receives the
   * bonds to remove. @returns the decision (fresh object), or null if nothing breaks (dents/promotion apply).
   */
  onContact(e: ImpactEvent, dt: number, alive: Uint8Array, bonds: number[]): BreakDecision | null {
    const a = this.breakable(e.chunkA, alive), b = this.breakable(e.chunkB, alive);
    if (!a && !b) return null;
    const F = e.forceVec, f = Math.hypot(F[0], F[1], F[2]);
    if (!(f > 0)) return null;
    const n: [number, number, number] = [F[0] / f, F[1] / f, F[2] / f];
    const vPre = (e.vPreA[0] - e.vPreB[0]) * n[0] + (e.vPreA[1] - e.vPreB[1]) * n[1] + (e.vPreA[2] - e.vPreB[2]) * n[2];
    if (vPre < this.minSpeed) return null;
    const iA = 1 / this.physics.bodyMass(e.bodyA), iB = 1 / this.physics.bodyMass(e.bodyB); // 0 for fixed
    if (iA + iB === 0) return null;
    const eRel = 0.5 * vPre * vPre / (iA + iB);
    // one body pair touching several chunks in a step (several collider-pair events) pays from one energy budget
    const key = e.bodyA < e.bodyB ? `${e.bodyA}|${e.bodyB}` : `${e.bodyB}|${e.bodyA}`;
    let spent = this.pairSpent.get(key) ?? 0;
    // break cheapest-first while the energy lasts
    let c0 = a ? e.chunkA : -1, c1 = b ? e.chunkB : -1;
    if (c0 < 0 || (c1 >= 0 && this.breakEnergy[c1] < this.breakEnergy[c0])) { const t = c0; c0 = c1; c1 = t; }
    const chunks: number[] = [];
    for (const c of [c0, c1]) {
      if (c < 0 || eRel - spent < this.breakEnergy[c]) continue;
      spent += this.breakEnergy[c];
      this.energy += this.breakEnergy[c];
      this.cutChunk(c, alive, bonds);
      chunks.push(c);
    }
    if (!chunks.length) return null;
    this.pairSpent.set(key, spent);
    return { chunks, n, vTarget: vPre * Math.sqrt(Math.max(0, 1 - spent / eRel)), maxJ: f * dt };
  }

  /**
   * Pass decision for a contact whose static side turns dynamic this step (joint-chain promotion): nothing breaks and
   * no energy is spent, so `handBack` undoes the bounce (v_n' = v_n^-) and Rapier resolves the hit against the moving
   * part next step, with its joints. @param e event; @param dt step (s). @returns decision, or null if not closing.
   */
  passThrough(e: ImpactEvent, dt: number): BreakDecision | null {
    const F = e.forceVec, f = Math.hypot(F[0], F[1], F[2]);
    if (!(f > 0)) return null;
    const n: [number, number, number] = [F[0] / f, F[1] / f, F[2] / f];
    const vPre = (e.vPreA[0] - e.vPreB[0]) * n[0] + (e.vPreA[1] - e.vPreB[1]) * n[1] + (e.vPreA[2] - e.vPreB[2]) * n[2];
    return vPre > 0 ? { chunks: [], n, vTarget: vPre, maxJ: f * dt } : null;
  }

  /**
   * Hands back absorbed impulse for decision d to bodies A, B (after any split; current masses/velocities):
   * dJ = clamp(mu (v_n' - v_n^+), 0, |P|), +dJ n on A, -dJ n on B. Measured now, so earlier hand-backs of the same
   * step are seen and never stack. @param bodyA/bodyB body handles (fixed bodies are skipped).
   */
  handBack(d: BreakDecision, bodyA: number, bodyB: number): void {
    const iA = 1 / this.physics.bodyMass(bodyA), iB = 1 / this.physics.bodyMass(bodyB);
    if (iA + iB === 0) return;
    const n = d.n, vPost = this.velAlong(bodyA, iA, n) - this.velAlong(bodyB, iB, n);
    const dJ = Math.min(Math.max(0, (d.vTarget - vPost) / (iA + iB)), d.maxJ);
    if (dJ === 0) return;
    const J = this.imp;
    J[0] = dJ * n[0]; J[1] = dJ * n[1]; J[2] = dJ * n[2];
    this.physics.applyImpulse(bodyA, J);
    J[0] = -J[0]; J[1] = -J[1]; J[2] = -J[2];
    this.physics.applyImpulse(bodyB, J);
  }

  /** @returns current CoM velocity of body b along unit n (3,) (0 for fixed bodies, invMass 0). */
  private velAlong(b: number, invMass: number, n: [number, number, number]): number {
    if (invMass === 0) return 0;
    const s = this.st;
    this.physics.readBodyStates([b], s);
    return s[7] * n[0] + s[8] * n[1] + s[9] * n[2];
  }

  /** @returns true if chunk c (-1 = not a chunk) can still break: breakable material, not yet cut, has an intact bond. */
  private breakable(c: number, alive: Uint8Array): boolean {
    if (c < 0 || this.cut[c] || this.breakEnergy[c] === Infinity) return false;
    for (let k = this.adj.adjOffset[c]; k < this.adj.adjOffset[c + 1]; k++) if (alive[this.adj.adjBond[k]]) return true;
    return false;
  }

  /** Marks c cut and queues its intact bonds. */
  private cutChunk(c: number, alive: Uint8Array, out: number[]): void {
    this.cut[c] = 1;
    this.broken++;
    for (let k = this.adj.adjOffset[c]; k < this.adj.adjOffset[c + 1]; k++) {
      const b = this.adj.adjBond[k];
      if (alive[b]) out.push(b);
    }
  }
}
