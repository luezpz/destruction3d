// Collision fracture under app conditions: async integrity (replies a few substeps late), 60 Hz x 4 substeps, balls
// thrown at every demo scene. Guards against the "explosion" / "ball slides around surfaces" failure modes.
import { beforeAll, describe, expect, it } from 'vitest';
import { createMaterialTable } from '../src/model/Material';
import { IntegrityCore, TickReply, TickRequest } from '../src/integrity/IntegrityCore';
import { IntegrityClient } from '../src/integrity/IntegrityClient';
import { RapierBackend, initRapier } from '../src/physics/RapierBackend';
import { packGroups } from '../src/physics/PhysicsBackend';
import { DestructionSim } from '../src/sim/DestructionSim';
import { SCENES, buildScene } from '../src/scenes';

beforeAll(async () => { await initRapier(); });

/** Inline core whose reply becomes visible only after `delay` polls (stands in for the worker's latency). */
class DelayedIntegrity implements IntegrityClient {
  readonly sync = false;
  private reply: TickReply | null = null;
  private wait = 0;
  constructor(private readonly core: IntegrityCore, private readonly delay: number) {}
  request(req: TickRequest): void { this.reply = this.core.tick(req); this.wait = this.delay; }
  poll(): TickReply | null { if (!this.reply || this.wait-- > 0) return null; const r = this.reply; this.reply = null; return r; }
  dispose(): void {}
}

const ALL = packGroups(0xffff, 0xffff), h = 1 / 240;

/** Runs a scene for `steps` substeps, throwing a 0.25 m steel ball at the target every 0.5 s. @returns metrics. */
function run(name: (typeof SCENES)[number], steps = 720) {
  const table = createMaterialTable(), demo = buildScene(name, table), model = demo.model;
  const physics = new RapierBackend({ gravity: [0, -9.81, 0], impactThreshold: 500, solverIterations: 16, pgsIterations: 1 });
  physics.createGround(0, ALL);
  const sim = new DestructionSim(model, table, physics, { client: new DelayedIntegrity(new IntegrityCore(model, table), 3) });
  const balls: { body: number; dir: number[] }[] = [];
  const st = new Float64Array(13);
  let maxDebris = 0, maxSide = 0;
  for (let k = 0; k < steps; k++) {
    if (k % 120 === 0) {
      const c = demo.camera, t = demo.target, d = [t[0] - c[0], t[1] - c[1], t[2] - c[2]], l = Math.hypot(...d);
      const dir = d.map((x) => x / l);
      balls.push({ body: physics.createSphere(0.25, [...c] as [number, number, number], [20 * dir[0], 20 * dir[1], 20 * dir[2]], 7850, ALL), dir });
    }
    physics.step(h); sim.tick(h);
    for (const isl of sim.handoff.islands) {
      if (!isl || isl.removed) continue;
      physics.readBodyStates([isl.body], st);
      maxDebris = Math.max(maxDebris, Math.hypot(st[7], st[8], st[9]));
    }
    for (const b of balls) {
      physics.readBodyStates([b.body], st);
      if (st[1] < 0.3) continue; // rolling on the ground: sideways is fine
      const v = [st[7], st[8], st[9]], along = v[0] * b.dir[0] + v[1] * b.dir[1] + v[2] * b.dir[2];
      const side = Math.hypot(v[0] - along * b.dir[0], v[1] - along * b.dir[1], v[2] - along * b.dir[2]);
      maxSide = Math.max(maxSide, side);
    }
  }
  physics.dispose();
  return { maxDebris, maxSide, breaks: sim.breaker.broken, bonds: sim.brokenTotal };
}

describe('collision fracture stress (async integrity, all demo scenes)', () => {
  for (const name of SCENES) {
    it(`${name}: no explosions, balls are not flung sideways`, () => {
      const r = run(name);
      if (process.env.STRESS_LOG) console.log(`${name.padEnd(20)} breaks ${String(r.breaks).padStart(4)} bonds ${String(r.bonds).padStart(5)} maxDebris ${r.maxDebris.toFixed(1)} m/s maxBallSide ${r.maxSide.toFixed(1)} m/s`);
      // balls fly at 20 m/s; debris faster than ~ball speed + a short fall means energy was created (the multi-event
      // and off-centre hand-back bugs reached 35-50 m/s)
      expect(r.maxDebris).toBeLessThan(23);
      expect(r.maxSide).toBeLessThan(15); // plain bounces off geometry reach ~10 m/s sideways (breaking off: 9.3)
    }, 60000);
  }
});
