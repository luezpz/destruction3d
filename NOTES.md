# NOTES

Memory for future sessions: status, architecture, design spec, decisions. Rules: `CLAUDE.md`. Design section refs are written "Design §N".

## Status
**Current milestone:** M4 — joint spike done (spherical joints + implicit angular motors chosen). In-house structure solver removed 2026-11-14 (external solver comes later, M3b).
**Next task:** remaining M4: browser-check the post-fracture damping and the detached-chain promotion on the steel demos (2026-11-17, tests only); eyeball the new joint demos in the app (visible bend on frame/tower/truss is small at default ball size), browser-check the damage-point dents (merged to master 2026-11-17, incl. `dent.offset/rotate/revealInner`), tune `oMax/jRef/tilt`. Open: bake ratchet of loop parts under sustained load above yield (NOTES). Promotion by stress ratio waits for the external solver (M3b). Consider evaluating Rapier 0.21 FEM soft bodies (NOTES) before investing more in joint chains.
**Blockers:** none

## Commands
- `npm run dev` (Vite, debug app at /), `npm run typecheck` (tsc 7, noEmit), `npm test` (vitest, Node), `npm run build`.
- `npm run test:gpu` (`scripts/skinCheck.mjs`): starts vite, opens `skincheck.html` (`src/dev/skinCheck.ts`) in system Chrome (headed, off-screen; `CHROME` env overrides path), GPU vs CPU skin for counts 1/8/32 on WebGPU and `?webgl` (WebGL2). Not part of `npm test` (needs a GPU + Chrome). 2026-11-15: max err ~1.3e-6 pos, ~1.2e-6 normal on both; tol 1e-4.

## Architecture map (`src/`)
- `model/` headless data: `ChunkModel.ts` (packed SoA format, flags, OUTER_FACE), `Material.ts` (records, presets, validation, derived interface materials), `boxGenerator.ts` (boxes -> ChunkModel: bonds from face overlap, face splitting + tags), `generators.ts` (grid, running-bond brick wall).
- `kernels/` pure typed-array math: `bondAdjacency.ts` (chunk -> bond CSR), `jointSpring.ts` (below).
- `integrity/` `StructureGraph.ts` (alive per bond, attached per chunk, adjacency, `removeBonds` -> tier 0 on the static part + `float*` components of floating parts that lost bonds (`Connectivity.collectComponents`, full BFS); seam for the external structure solver), `connectivity.ts` (tier 0 early-exit BFS -> detached components), `loads.ts` (chunk masses, own rotational inertia).
- `physics/` `PhysicsBackend.ts` (interface, group bits, packGroups), `RapierBackend.ts` (only Rapier importer), `fromModel.ts` (Float64 -> Float32 hull conversion).
- `kernels/jointSpring.ts` joint error `e = [dx, phi]` in the bond frame from body states (13 doubles/body), spring wrench, stiffness cap (Design §4b spike).
- `integrity/islandMass.ts` mass/COM/principal inertia (Jacobi) from rest cell volumes.
- `integrity/IntegrityCore.ts` message handler (TickRequest{remove} -> TickReply: broken, detached, version); `IntegrityClient.ts` (`InlineIntegrity`, `WorkerIntegrity`); `integrity.worker.ts` entry.
- `sim/` glue between integrity and physics: `IslandHandoff.ts` (static body + per-chunk colliders, detach components -> dynamic islands, `refracture` splits islands whose chunks land in 2+ float components: new bodies at the old pose + rigid velocity, first piece reuses the island index; grace -> debris groups; islands store per-chunk body-frame poses `local` (7k), chunks keep a static pose `setPos/setQ` = rest until demoted, new islands start from it; `attach` = reverse of detach (demotion), entries stay with `removed`), `DestructionSim.ts` (per-physics-step `tick`: poll reply -> breaks/handoff, send queued removals when none in flight; `brokenLog`, `isStatic`, `alive` per bond; drains impacts -> `breaker.onContact` first, else `chains.onImpact` + dents), `CollisionBreaker.ts` (universal collision fracture, every contact-force event, all at the CoM: closing speed from backend pre-step velocities, E_rel = 1/2 mu v_n^2 vs E_break(c) = s_E V_c (1/2 sigmaT eps_el + sigmaY eps_pl), cheapest touching chunk first, one energy budget per body pair per step; decisions applied after the drain: cut chunks freed at once by `IslandHandoff.splitChunk` (static -> single-chunk island, island -> refracture [rest, c]), then `handBack` raises the closing speed to v_n sqrt(1 - spent/E_rel) with dJ = clamp(mu (v' - v_now), 0, |P|) measured at apply time; supports + SKINNED chunks never break; a contact that promotes a joint-chain part (`chains.onImpact` true) gets `passThrough` (no break, v_n' = v_n^-: Rapier's bounce off the still-static part is undone and the next step resolves the hit against the moving jointed part); `minSpeed` gate; `sim.dents` toggles fake dents), `JointChains.ts` (Design §4b: promote part -> single-chunk islands + spring joints; per step: joint errors via `computeJointSprings` with k = 0, `updateJointPlasticity`, `setSpringTarget` on yield, break -> `removeBonds`; drops joints whose bond integrity removed or whose static anchor detached; demotion of anchored parts at rest for `demoteTime` (1 s): reachable bodies -> `handoff.attach`, joints dropped without bond removal, eps_p + targets kept per bond and restored on re-promotion; demoted body slots point at the static body).
- `kernels/jointPlasticity.ts` corner von Mises (N = V = 0) -> plastic target return + eps_p, elastic / brittle break rules.
- `model/skinWeights.ts` Design §4 weights: entries `id<<16|w16`, per-vertex offset/count/capacity, `chunkVerts` (chunk -> influenced verts, also §5 hull input), `reweightBroken` (in place, fills `dirtyVerts`). Support R_j = rho * bounding radius of chunk j's verts (rho > 1 keeps the owner); only SKINNED chunks blend, others count == 1. Ties sorted by id so seam copies match exactly.
- `kernels/skin.ts` CPU skin reference; pose buffer 8 doubles/chunk `[a.xyz, 0, q.xyzw]`, `a = x - q c` (= the 2 RGBA32F texels).
- `integrity/DentField.ts` fake dents (Design §4 offsets): per-chunk damage points D, offset along the hit dir + random-axis tilt, hit chunk only, never breaks bonds, per-material `offset`/`rotate` flags; owned by `IntegrityCore` (request `impacts`, IMPACT_STRIDE 9; reply `poseChunks/poses`). Main: `DestructionSim.queueDent` filters by `dent.minForce`, reply poses -> `handoff.setStaticPose` (detach then starts from the dented pose). `main.ts` `syncRender` reveals inner faces of dented chunks when `dent.revealInner`.
- `render/islandPose.ts` `writeIslandPoses`: body pose * `local` -> skin poses. Demoted chunks: `main.ts` drains `handoff.staticPoseDirty` once. `model/quat.ts` quat rotate/multiply.
- `render/skinMaterial.ts` pose texture (RGBA32F, 2 texels/chunk, width 2048) + entry texture (R32UI), `skinRange` attribute, TSL `skinPositionNode` (mirror of `kernels/skin.ts`), `uploadDirtyWeights`.
- `ui/` `hud.ts` + `style.css`: HUD overlay (scene, fps, sim ms, key counters), hint bar, lil-gui theme. Keys: click/space ball, shift+click cut, P pause, R reset.
- `main.ts` app: physics.step -> sim.tick per fixed step; `syncRender` reveals broken faces, reweights skin around new breaks (own `alive` copy from `brokenLog`), writes island poses to the pose texture.
- `render/` `cullBuffers.ts` (headless outer/inner index buffers, append-only reveal), `chunkMesh.ts` (two meshes sharing one vertex buffer), `bondLines.ts` (debug lines, intact green / broken hidden).
- `scenes.ts` 19 demo scenes (box lists, mixed materials, `carve` for openings) + per-scene camera; `SCENE_STRENGTH` = per-scene default strength multipliers that `main.ts` `selectScene` loads into the sliders (joint demos weaken steel so a ball bends it; ball speed slider goes to 120 m/s). Joint (`deform: 'joints'`) demos: 'steel plate', 'steel cantilever', 'steel beam', 'steel truss bridge' (Vierendeel, no diagonals: boxes are axis-aligned), 'steel frame', 'steel tower', 'steel pole', 'rubber curtain'; `main.ts` debug app (GUI: scene, visibility, break random bonds, stats; click/space = ball aimed at cursor with gravity lead, shift+click = cut all bonds of the static chunk under the cursor). GUI folders: debug view (outer/inner faces, bond lines), sim (sync = `InlineIntegrity`, pause/step, time scale, physics Hz, substeps; gravity/impact threshold/Rapier iters reload), material multipliers (`scaleMaterials`, reload; only density matters until the external solver), stats (version, replies, latency). `window.demo` exposes `cur` (model/sim/physics), `params`, `matScale`, `load`, `selectScene`, `throwBall` for console/automated checks.
- `src/dev/` browser-only check pages (`skinCheck.ts` <- `skincheck.html`), driven by `scripts/*.mjs`; not in the app build.
- `tests/` model, culling, Rapier facts, graph (tier 0), integrityCore (boundary + slow client), handoff, islandMass/Pose, jointSpring, skin, jointChains, jointScenes (joint demos: rest holds, ball bends).

## Conventions
- Chunk ids < 65536 per mesh (generator throws). Bond a = lower side; normal points a -> b; frame `[n t1 t2]` right-handed.
- Render mesh is unwelded per face (flat normals); a box face is split along neighbour overlap rectangles so each triangle is outer or exactly one bond's cut face.
- Hull points and render verts are chunk-local (relative to `chunkPos`).
- `createMaterialTable` copies the input array; `interfaceMaterial` appends derived `a|b` records (series E, weaker strengths, plastic only if both sides plastic).

## Rapier API facts (@dimforge/rapier3d-compat 0.21, verified in tests/rapier.test.ts)
- No collider re-parenting (`Collider.setParent` absent) -> detach = remove + recreate from cached hull.
- `ColliderDesc.convexMesh(points, indices)` accepts supplied indices; `collider.setShape(desc.shape)` works. Whether it skips hull computation internally is NOT verified (perf check in M5).
- Overlapping colliders of one body: no contacts. Fixed-vs-fixed overlap: no contacts. Dynamic-vs-fixed overlap with shared groups: contacts (positive control).
- Grace group (membership GRACE, filter DEBRIS only) lets a new island overlap the structure without push-out.
- Contact force events: `ActiveEvents.CONTACT_FORCE_EVENTS` + threshold; `drainContactForceEvents` gives total force magnitude + max-force direction.
- `narrowPhase.contactPair(c1, c2, world.bodies, cb)` — `bodies` arg is required in 0.21 (missing it silently never calls cb).
- `GenericImpulseJoint` has **no per-axis motor API** in JS; `SphericalImpulseJoint` does (AngX/Y/Z `configureMotorPosition`, `MotorModel.ForceBased`). But `createImpulseJoint(JointData.spherical(..))` returns a `GenericImpulseJoint` object (raw type reports Generic): wrap the handle with `new RAPIER.SphericalImpulseJoint(world.impulseJoints.raw, world.bodies, handle)` (done in `RapierBackend.sphericalJoint`). `setFrameX1/X2` exist on all impulse joints.

## M4 joint spike (tests/jointSpring.test.ts, 2026-11-14): steel cantilever, 10 cubes h=0.1 m (7.85 kg), 10 s
- **Chosen: spherical joint (translation locked) + 3 implicit angular motors** (`createSpringJoint`). 4 substeps @60 Hz, 16 solver iters, k = raw GJ/L, EI/L x 1e-2: sag 8 mm (EB 0.04 mm), no drift, root moment k*phi within ~9% of statics. At k x1 the sag (1.3 mm) is solver error, so k*phi overestimates the moment 6x; at x1e-3/1e-4 the readout drops (0.79/0.45). Keep the motor compliance just above the solver floor.
- Chains vs theory (`tests/jointChains.test.ts` "hinge", 2026-11-15, same cantilever, production JointChains with damping/sleep): sim 5.9 mm, discrete hinge chain at k x1e-2 4.35 mm (sim 1.36x softer = solver compliance), continuum EB at E x1e-2 3.8 mm, raw EB 0.038 mm. So the effective stiffness is ~0.0074 E; scale material E/sigmaY expectations accordingly.
- Solver floor (sag of fixed joints): 0.50 m (4 it, 1 sub), 5.1 cm (4 it, 4 sub), 4.5 cm (16 it, 1 sub), 3.2 mm (16 it, 4 sub). Chains need substeps + iterations.
- (a) explicit springs (`kernels/jointSpring.ts`): kernel exact on short chains (sag within 1% of capped statics) but the stability cap `k <= m omega^2` makes steel ~1e5x too soft (predicted 6 m sag at 4 substeps), long chains barely damp (joint-relative damping leaves low modes undamped). A translational spring at lever l also needs `k l^2 <= I omega^2`, else bodies spin up. Kept only as the error/moment readout (k = motor stiffness, c = 0).
- (b) fixed joints: stiffest at few iterations, but error -> stress is uncalibrated (Design §4b) and there is no motor to move a plastic rest.
- Axial/shear bond forces are not measured by (c) (translation locked); metals fail by bending first; get N/V from equilibrium if needed.
- **Handles are non-integer doubles** (index+generation encoded): never store them in Int32/Uint32 arrays (silently removes the wrong collider). Use Float64Array / plain numbers.
- `setAdditionalMassProperties` takes effect only after a `step`; the principal-frame quaternion (`IslandDesc.inertiaFrame`) is honoured (tested).
- Contact-force event `dir` points from collider1 (A) to collider2 (B): force on B = +dir F, on A = -dir F. Which collider is "1" depends on creation order, so always use the sign rule.
- Mass: colliders at density 0 + `RigidBodyDesc.setAdditionalMassProperties` gives exact island mass.

## Removed structure solver (2026-11-14; code in git history before this date) — lessons for the external one
- PCG on cracked lattices: restarting every request with a ~2 ms budget never converged; continue while loads are unchanged. Warm start helps only at loose tol (~1e-4).
- Single-step impact peaks get filtered away by a ratio low-pass; hold peaks >= ~0.05 s and until a converged solve.
- Rapier contact-force `dir` points collider1 -> collider2 (force on B = +dir F); `EventQueue(true)` auto-drains each step, so read impacts after every substep.
- Undamaged regular grids converge unrealistically fast; benchmark with irregular damage.

## Joint-chain perf (2026-11-15, tests/chainPerf.test.ts; 3 cm steel plate, 90 bodies, 171 joints, 4 substeps)
- Cost is Rapier, not us: awake plate 13-16 ms/frame at 16 iters (our loop ~0.5 ms). Solver iterations drive it: (iters x substeps) 16x4 11.2 ms, 32x2 9.1, 32x1 4.8 (cantilever sag 11.2 vs 7.9 mm), 8x4 5.4 (14.7 mm). Kept 16x4.
- Fix = settle + sleep: absolute body damping (2/s linear + 2/s angular while the part is anchored; any body not reachable from the static body through alive joints — whole part or a fragment of a still-anchored part, checked after each joint break — gets `freeLinearDamping/freeAngularDamping` 0 / 0.05 once, 2026-11-17) + own rest test (all part bodies < 0.005 m/s, 0.02 rad/s for 0.25 s -> `sleepBodies`). Hit plate: 17, 9, 3.5, 0.5 ms/frame over 4 s, asleep after.
- Damping slows convergence of the joint error correction: steel cantilever sag 5.9 mm (8.2 undamped; 7.1 kept awake 10 s, still creeping). Moments from `k phi` read up to ~30% low; accepted (sag is exaggerated ~200x anyway).
- Ball-hit soft plate (strength x0.01) jittered forever (one body at 10-30 rad/s, anchor gaps 1-10 mm, no contacts, plastic rest angles static). Causes, in order: (1) same-part diagonal contacts -> part collision groups; (2) loop incompatibility of independent plastic targets with locked anchors -> rest-state bake per part after 0.1 s without yield. Tried and dropped: per-joint anchor slip on the raw gap (anchors random-walk with jitter, energy in), on the low-passed gap (keeps firing: rotations are incompatible, not translations), counting slip as eps_p (70/171 joints tore).
- Verified: our `phi` matches Rapier's spherical motor angle at equilibrium within 1% up to 0.4 rad per axis (small-angle 2 vec(q)).
- Rest-state bake ratchet (2026-11-15): baking a part under a sustained load above yield makes it stress-free, gravity drops it again (dynamic ~2x), it re-yields, re-bakes: a softSteel cantilever never rested (42 bakes in 8 s). Fix: bake only parts with loops (alive joints > alive bodies; static body = one node). Loop parts under sustained load above yield (fixed-fixed beams, horizontal plates) still ratchet: open.
- Demotion (cantilever, `tests/jointChains.test.ts`): pushed holdSteel cantilever demotes 1.24 s after the push at 12.3 cm sag; re-promoted it stays at 12.3 cm (stored targets, anchors re-closed); a root cut detaches it as one island at the frozen pose.
- Rapier 0.21 ships FEM soft bodies (`dynamics/soft_body.d.ts`: particles, pinning, damage flags, self contacts). Not evaluated; a possible alternative to joint chains for `joints` materials.

## Gotchas
- Static `joints` chunks detached by integrity (cut loose with non-joint neighbours, or a demoted part cut) are promoted as a joint chain (`DestructionSim.apply`: promote while the rest of the component is still static, then detach), not a rigid island. Joints whose static chunk leaves the structure are **re-bound** onto the rigid island now holding it (a "carrier", `JointChains.rebind`; also after the carrier island splits), not dropped: a curtain stays hanging on its falling lintel (free damping, no demotion; `tests/chainCarrier.test.ts`). Only a carrier chunk that is itself cut loose or a dead island breaks the joint.
- Collision breaking uses energy, not contact force: Rapier's event force is ~impulse/dt (one-step stop), so a force threshold changes with dt. The bounce is undone right after its step, not prevented. Default E_break is small (concrete 0.4 m cube ~ 336 J); tune `energyScale`. Events need `impactThreshold` (N) low enough to fire.
- Collision-break bugs found 2026-11-16 (tests/collisionStress.test.ts guards them, async integrity + all scenes): (1) "pass through pending cuts" with the worker let balls sink into still-static chunks (slid along surfaces) and islands into the ground (overlap explosions) -> free cut chunks immediately instead; (2) one body touching k chunks = k events, each handing back the full impulse -> k x speed (50 m/s debris) -> per-pair energy budget + hand-back measured after earlier ones; (3) hand-back at the contact point added spin the CoM mass ignores -> CoM only. Velocities reconstructed from the event impulse (v+ - P/m) include penetration recovery: use pre-step snapshots.
- Dents (2026-11-17 damage-point model, no cracks): one contact arrives as many substep events; saturation is per hit chunk on accumulated impulse, so splitting doesn't matter. Contact point = chunk centre (events have none). Debris landing on the wall also dents (cascades at high ball speed: 15 m/s ball, 3 hits -> ~260 breaks); tune via `dent` records. Concrete jRef 2000 N s: 8 m/s ball (r 0.3) dents ~1 cm (measured with the old crack model; recheck). Dents no longer break bonds, so the cascade note applies only to collision breaks.
- Joint to the static body disables contacts between that chain body and ALL static colliders (one static body holds the structure). Fine for flush neighbours; a promoted part can pass through the static structure it is jointed to.
- Zombie sleep (fixed 2026-11-17, `tests/chainGroundSleep.test.ts`): `body.sleep()` on single bodies (our `sleepBodies`) bypasses Rapier's island grouping; when a contact later wakes part of a slept chain, the rest stays flagged asleep but in the active set: gravity still integrates it, its contacts with fixed bodies are skipped, so it sinks through the ground (dropped curtain strips). `RapierBackend.step` wakes any sleeping body found in the active set.
- Sleeping Rapier bodies ignore world gravity changes (tests wake them); motor target changes may not wake them either. Heavily damped bodies creep slowly enough that Rapier's own sleep can freeze them short of equilibrium.
- Joint chains are inextensible (translation locked): a rubber/steel sheet fixed on all edges is a taut membrane and doesn't deflect (a first 'rubber sheet' demo bounced the ball off). Use free-hanging or cantilevered parts ('rubber curtain').
- Joint-demo strengths (2026-11-17, `SCENE_STRENGTH`): a part must hold its own weight (bending stress ~ 3 rho g L^2 / h for a cantilever, 0.75 rho g L^2 / h spanning) below sigmaY or it yields at rest/ratchets; ball (r 0.25, 20 m/s) then yields a few dozen joints but moves stiff frames/towers only ~1-2 cm: raise the ball size/speed in the GUI to see more.
- Steel at real strength never yields from ball impacts (ratio ~0.02 on a 3 cm plate); use the strength multiplier (x0.01) to see plastic bending in the demo.
- Rapier `EventQueue(true)` auto-drains on every `world.step`: with substeps, call `sim.tick` after each substep or impacts are lost.
- Bond lines use additive blending: black = hidden (broken / below threshold).
- Browser check without a user: playwright-core + system Chrome (`--enable-unsafe-webgpu`, headed, off-screen window) drives `npm run dev`; ad-hoc scene scripts stay outside the repo, automated checks go in `scripts/` (playwright-core is a devDependency).
- GPU readback harness: render `THREE.Points` with `vertexNode` placing vertex v at texel v of a V x 1 float target, keep `positionNode` set (it still runs and assigns `positionLocal`/`normalLocal`; reading them via `varying` gives the real-path values). The output stage clamps negatives to 0 -> encode with an offset. Bare `THREE.Camera` crashes `render` (needs `updateProjectionMatrix`).
- tsconfig lib has no `WebWorker` (it breaks DOM event typing). Don't use `/// <reference lib="webworker" />` either (applies program-wide); type the worker scope minimally as in `integrity.worker.ts`.
- `ChunkModel` and `MaterialTable` (Map, Infinity) structured-clone fine; worker init clones them once.
- Uint DataTexture: `RedIntegerFormat` + `UnsignedIntType`; do NOT set `internalFormat = 'R32UI'` (WebGPU texture creation fails, mesh invisible). Wrap `textureLoad(..).x` in `uint()` for TS types.
- Browser check harness lives in `~/d3check` (playwright-core; `vite --port 5179`, `window.demo`).
- WebGPU rejects zero-size buffers: inner index buffer capacity kept >= 3.
- Bundle is ~5 MB (three/webgpu + Rapier base64 wasm); fine for dev, code-split later (M7).

# Design spec (formerly the Design spec)

## destruction3d — plan v2 (Rapier does rigid bodies; we write the destruction layer)

Standalone: no imports from other projects (can be lifted out). Rapier (`@dimforge/rapier3d-compat`, Havok later)
owns rigid bodies, compounds, contacts, sleeping. Hide it behind a thin `PhysicsBackend` interface
(`createStaticBody`/`createStaticChunk`, `removeCollider`, `createDynamicIsland`/`removeBody`, `setChunkShape`, `createJoint`/`createSpringJoint`/`setSpringTarget`/`removeJoint`,
`readBodyPoses`/`readBodyStates`, `applyWrenches`, `setCollisionGroups`, `drainImpactEvents`; source of truth: `physics/PhysicsBackend.ts`) so Havok is a swap.
We implement: **integrity simulator** (+ data-driven materials), **island splitting**, **many-bone skinning**, **fast hull refit**, **culled rendering**; Voronoi is last.
Out of scope for now: audio, dust/debris particles, save/load, networking, terrain destruction.

### Setup (M1)

Vite + TypeScript + three.js (WebGPU/TSL) + lil-gui + Rapier, own `index.html`. Layout: `model/` (ChunkModel, Material table, generators), `kernels/` (pure typed-array math),
`integrity/` (structure graph, connectivity, worker entry; external structure solver plugs in here), `physics/` (PhysicsBackend + Rapier impl), `render/` (skin shader, culling buffers), `tests/` (headless, no three.js/Rapier where possible).
Scripts: `dev`, `typecheck`, `test`, `build`. Keep a terse `NOTES.md` (architecture map, commands, decisions, gotchas) updated with each change. Units: SI, except E/H in GPa and strengths/stresses in MPa (converted to Pa inside kernels).

### Data flow

```
main thread                                   Worker (integrity)
Rapier: static chunk colliders + debris  --removed bonds, impact loads*-->  [external structure solver*] -> bonds to break
        ^                                                                    StructureGraph + Connectivity (components)
        | create/remove colliders, refit shapes                              |
        +---- islands, broken bonds, chunk poses* (static graph + offsets) <--+
renderer: pose DataTexture + many-bone skinning shader
          pose sources merged on the main thread: static/"offsets" chunks from the worker, chunks on dynamic bodies
          (debris islands, promoted joint chains) from Rapier (body pose o local)
```
`*` = not wired yet: impact loads and the solver arrive in M3b, worker chunk poses with fake offsets in M4. Today the worker gets removals and returns breaks + detached components, plus the components of floating parts that lost bonds (island re-fracture).
Worker messages use double-buffered transferable typed arrays (no SharedArrayBuffer, so no COOP/COEP headers needed).
A 1-3 frame latency is fine for fake destruction.

### 1. Data model (`ChunkModel`, headless, no three.js/Rapier)

One format, produced by every generator (grid/bricks first, Voronoi and glTF later):

```
chunks: pos(3), rest hull points + face indices, volume, material id (index into the Material table, section 2b), support flag, "skinned" flag (= material.deform === 'joints'),
        face tags (outer | inner+bondId), surface flag (has any outer face)                                 // packed arrays
bonds : a, b, area, centroid(3), normal(3), tangents, I1, I2, J, face-polygon corners, material id (interface material or derived), reinforcement
```
Float64 in the solver, Float32 only at the Rapier/GPU boundary.

### 2. Integrity simulator (background)

**Status (2026-11-14): the in-house structure solver (RBSN bond lattice, block-Jacobi PCG, tier-1 failure ratios + damage) was removed**:
results and feel were not what we wanted. An **external structure solver** (developed separately) will be integrated later. Until then bonds break only
by scripted removal (cuts, explosions, picking) and tier 0 connectivity drops whatever loses support.

Kept (solver-independent):
- **Structure graph** (`StructureGraph`): `ChunkModel` + per-bond `alive` flags + chunk -> bond adjacency (CSR) + per-chunk `attached` flag.
- **Tier 0, connectivity** (instant, O(local)): on a bond break, BFS from both ends, stop as soon as a support (or a proven-grounded chunk) is reached; unsupported components go to section 3.
- Message boundary (section 7): `IntegrityCore` (inline or worker) takes removals, returns broken bonds, detached components and a graph version.

**External solver contract (to be fixed when it lands)**
- Input: the graph (chunks: pos, volume, mass `m_i = rho V_i`, material id, support flag; bonds: a, b, area, centroid, normal, tangents, `I1, I2, J`, face corners,
  material id incl. derived interface materials), the `alive` mask, the `Material` table (section 2b), and loads (gravity; impacts from `drainImpactEvents`).
- Output: bonds to break (and optionally per-bond stress/utilisation for display, per-chunk displacements for `offsets` render poses, section 4).
- It runs inside `IntegrityCore` (worker side) so the frame never waits; breaks it reports go through `StructureGraph.removeBonds` -> tier 0 -> handoff, as scripted cuts do.
- Prior approach, for reference (git history before 2026-11-14): 6-DOF bond stiffness `K_b`, CSR of 6x6 blocks, warm-started PCG, corner-stress failure ratios, low-passed damage
  with propagation, impact peaks held ~50 ms. Lessons in NOTES "Removed structure solver".

### 2b. Materials (data-driven; many types)

The strength/damage fields below are data for the (external) structure solver and for joint chains (4b: `E`, `nu`, `sigmaY`, `H`, `epsPlasticMax`); with the in-house solver removed, only density (masses), `deform`, `hull`, `render` and `reinforcement` are read by the app today.

A material is a record of behaviour parameters, not a kind. Presets are just data (concrete, steel, aluminium, glass, wood, brick, rubber, clay, ice, drywall, ...).

```
Material {
  density, E (GPa), nu,
  // strength (MPa)
  sigmaT, sigmaC, tauC, mu,            // tension, compression, cohesion + friction for shear; sigmaC = 0 -> sigmaT
  sigmaY,                               // yield; 0 = no real plasticity (fake `offsets` dents still apply)
  hardening H (GPa), epsPlasticMax,     // only used when sigmaY > 0: ductile break at this plastic strain
  epsElasticMax,                        // only used when sigmaY == 0: strain limit for elastic-to-break (rubber)
  damageRate, filterTime,               // damage per second per unit of overload (Infinity = instant), low-pass window
  propagation,                          // on a break, add this much damage to neighbour bonds (glass/ice shatter, 0 = local)
  grain: { axis, strengthRatio } | none // anisotropy (wood): bond strength scales with angle between bond normal and grain axis
  deform: 'none' | 'offsets' | 'joints',// presentation/physics of deformation: nothing, fake impact offsets (4), real joint chain (4b)
  hull: 'rigid' | 'refit',              // collider handling for deforming parts (5)
  render: { outer, inner },             // materials for outer faces and for revealed fracture faces (6)
  reinforcement?: { fraction, material } // e.g. rebar: a second bond with its own material that survives after this one fails
}
```
Axes, not names: `sigmaY > 0` = plastic (metals, clay, aluminium); `sigmaY = 0` with a low `epsElasticMax` = brittle (concrete, glass, ice); `sigmaY = 0` with a high `epsElasticMax` = elastic (rubber, bounces back, no permanent set).

Corner stresses (shared by the solver and 4b): bond load `[N, V1, V2, Tq, M1, M2]` in the bond frame (normal `n`, tangents `t1, t2`); at face corner `c` with
in-plane coords `(y_c, z_c)` relative to the centroid, `sigma_c = N/A + M1 z_c/I1 - M2 y_c/I2`, `tau_c = sqrt(V1^2 + V2^2)/A + |Tq| sqrt(y_c^2 + z_c^2)/J`;
`sigma_max/min` and `tau` are taken over the corners.
Failure ratio of a bond = max over the criteria its material enables:
- tension `sigma_max/sigmaT`, compression `-sigma_min/sigmaC`, shear `tau/(tauC + mu sigma_compr)` (always);
- plastic (`sigmaY > 0`): `sigma_v/(sigmaY + H eps_p)` with von Mises `sigma_v = sqrt(sigma^2 + 3 tau^2)` at the worst corner; above 1 the rest target moves (plasticity), break at `eps_p > epsPlasticMax`;
- elastic (`sigmaY == 0`): `eps / epsElasticMax`, with `eps = max_corner |sigma| / E` (axial + bending fibre strain).
Invalid combinations are rejected when the table loads: `hull: 'refit'` needs `deform: 'joints'` (only bodies with skinned chunks change shape).
Where plasticity actually *happens* depends on `deform`: `joints` -> the promoted joint chain updates its rest targets (4b); in the static graph a plastic bond with ratio > ~0.7 triggers
promotion (4b; needs solver ratios, so until M3b promotion is by impact or script only). `offsets`/`none` with `sigmaY > 0` (e.g. clay or soft stone as dents) have no rest-target update: ratio > 1 grows damage, and `epsPlasticMax` is read as a damage limit.
Then damage as above. `propagation`: when a bond breaks, `d_neighbour += propagation` for its adjacent bonds, which makes glass shatter in a chain; 0 keeps failure local.
Grain: strength of a bond is multiplied by `lerp(1, strengthRatio, 1 - |n . axis|)`, so wood splits along the grain.
Bond materials: a bond joining chunks of different materials uses an explicit interface material if the generator provides one (mortar between bricks, steel plate on concrete);
otherwise stiffness in series `1/E = 1/E_a + 1/E_b` and the *weaker* strengths.
Which deformation path a part uses comes from `deform` (and plastic vs elastic from `sigmaY`), so a new material needs only a new record, plus a test row in the material matrix test (every preset: a cantilever under load, a compressed column, an impacted wall; no NaN, expected failure mode). The material matrix test was removed with the solver; it returns with the external solver.

### 3. Island splitting -> Rapier

This is **connected-component labeling** (union-find / BFS over intact bonds), not graph coloring (which assigns
different colors to neighbours). Run after each batch of removed bonds, local to the affected components only.

1. Label components. A component that contains a support stays the static structure (one fixed Rapier body holding all its chunk colliders, so joints can anchor to it).
   Settled debris never becomes a support (it stays a sleeping dynamic body).
   Reinforcement: a bond whose base material failed but whose `reinforcement` survives counts as intact for tier 0 *only as a hanger*: if a component is connected to a support
   solely through such bonds, it still becomes a dynamic body, attached by a ball joint (break force from the reinforcement material), contacts disabled on that joint. That is the dangling slab.
2. Every other component becomes one dynamic body: `RigidBody.dynamic` at the component COM, one convex collider per chunk
   (local translation = chunk pos - COM). Mass: set explicitly from the rest cell volumes x density (colliders at density 0 + `setAdditionalMassProperties`); hulls overlap, so Rapier's per-hull volume would double count. Inertia from the same cell volumes (sum of cell inertia + parallel axis about the island COM).
3. Velocity: 0 when it comes from the static structure; when splitting a moving debris island, `v + w x (com_child - com_parent)`.
4. Remove the detached chunks' old colliders from the static structure (`removeCollider`). Rapier JS cannot re-parent a collider (verified in M1, NOTES "Rapier API facts"),
   so cache each chunk's hull points + indices and recreate (see section 5: indices are supplied; whether that skips quickhull is checked in M5).
5. Each new island keeps its sub-graph in the worker, so it can fracture again (not implemented yet: handoff skips components that are already debris).
Test: mass and momentum conserved across a split; labeling of a known removal pattern; a one-support-removed wall detaches exactly its dependents.

### 4. Many-bone skinning for touching cells (mainly `deform: 'joints'`) and fake deformation (`deform: 'offsets'`)

Why 4 bones fail: smooth bending of a sheet of touching cells needs every cell within the kernel radius. A grid vertex
with a 1.6-cell support touches 8 cells (27 with a wider one); a Voronoi cell has ~14 neighbours. So the influence count is
**variable per vertex with no fixed K** (see "Variable-length weights"); typical counts are 1 (`offsets`/`none` materials) to ~8-16 (`joints` materials), soft cap 32.

- **Pose per chunk**: for chunks on Rapier bodies (promoted joint chains, debris islands) the pose is read straight from the body (section 4b); for the rest it comes from the worker
  (rest pose + offsets, 4). The earlier idea (forward kinematics over the bond tree with `phi_el + phi_p`) is dropped: it was the least-proven piece and needs a separate hinge model.
  The weighting scheme applies to any material with skinned vertices; chunk ids are 16 bit (<= 65535 chunks per mesh; split the mesh or widen the entry to two `uint32` beyond that).
- **Weights**: compact kernel `phi(s) = (1 - s^2)^3`, `s = |p - c_j| / R_j`, `R_j = rho * cellRadius_j` (rho ~1.6-2.2),
  normalized over the support `S(p)` (spatial hash), then drop weights < ~1e-3 (and beyond the soft cap, largest kept) and renormalize (partition of unity).
- **Topology-aware**: keep only cells connected to the vertex's owner cell through *intact bonds inside `S(p)`*. Two panels
  that happen to be close do not blend. A broken bond makes the two sides drop each other's bones, so the tear is automatic.
  Only vertices whose `S(p)` contains the broken bond are re-weighted (local), then their buffer slice is re-uploaded.
- **Seams**: pre-split the mesh at bond planes with duplicated seam vertices (each copy owned by one side). While intact both
  copies get the identical set (same `S(p)`, same component), so there is no visible crack; after a break each copy only
  sees its own side. No runtime mesh cutting, no triangle stretching across the break.
- **Variable-length weights (Unity-style "unlimited influences", as I recall its design)**: no fixed slots per vertex. One flat array of
  `(boneIndex, weight)` entries for the whole mesh plus a per-vertex count (`bonesPerVertex`, a byte, hence the 255 limit); a vertex's start is the prefix sum
  of the counts (CSR layout). The skinning pass loops over that vertex's range. We do the same:
  - `entries: Uint32Array`, each `id << 16 | w16` (16-bit chunk id, unorm16 weight; no 255 limit since we store the count in 16+ bits);
  - per-vertex attribute `uvec2 (offset, count)`;
  - skin: `p' = sum_{e in [offset, offset+count)} w_e (q_e p + a_e)`, `n' = normalize(sum w_e q_e n)`; a `count == 1` fast path skips the loop (all `offsets`/`none` vertices).
  - Break updates are **in place**: a break only removes influences (weights are topology-masked), so the new count <= the old, rewritten inside the vertex's slot
    (capacity = initial count). No reallocation; upload only the dirty vertex ranges. Plastic bending never changes weights.
  - Delivery: WebGPU storage buffer (read-only, vertex stage) via TSL; WebGL2 fallback = the same arrays in a data texture with `texelFetch`.
    Optional compute pre-pass that writes skinned positions/normals to a buffer (what Unity's GPU skinning does) when depth/shadow/colour passes would otherwise skin 2-3x;
    only worth it if profiling says so.
  - Divergent loop counts: neighbouring vertices have similar counts, so waves stay coherent; cost is `sum(count)` texel pairs per frame, bounded by the soft cap.
  Chunk transforms in an `RGBA32F` DataTexture (or storage buffer), 2 texels/chunk: `a_i = x_i - q_i c_i` (rest centre `c_i`), `q_i`. One draw call, no bone-count limit; the texture is tiny (1000 chunks = 32 KB), re-upload only when poses change.
- **CPU reference** of the same formulas for tests and for small low-poly parts.
- **Fake deformation (`deform: 'offsets'`, e.g. concrete, brick, stone, plaster)** (decision: no real nonlinear solve; no skinning, every such vertex has `count == 1`, so chunks just move rigidly).
  **(2026-11-17: superseded by the damage-point model below; the falloff/crack text in this bullet is historical.)** Each chunk keeps a permanent visual offset `o_i: (3,)` and tilt `th_i: (3,)` on top of its rest pose; render pose is
  `x_i = x_i0 + amplify * u_el,i + o_i`, `q_i = exp(th_i + amplify * th_el,i) q_i0` (both world-frame rotation vectors, small), with `u_el` the structure solver's displacement (sagging slabs; 0 until the external solver provides it) and `o, th` from impacts
  (the total offset is clamped to `o_max` so the render pose never strays far from the collider):
  - Impact at point `p`, unit direction `n`, impulse `J`: for chunks within graph radius `R`, `do_i = s_sat(J) phi(g_i/R) n`, `dth_i = k_t phi(g_i/R) (n x (c_i - p)) / R`,
    where `g_i` = bond-graph distance from the hit chunk, `phi(s) = (1 - s^2)^3`, `s_sat = o_max (1 - exp(-J/J_ref))` (saturates, so repeated hits dent less; implemented as the increment along this curve of the impulse accumulated per hit chunk, so one contact split over many substep events dents like one hit), `o_max` ~5-10% of chunk size.
  - Cracks fall out of the offset gradient: per bond, opening `w_b = (o_j - o_i) . n_b` (shear slip from the tangential part); `d_b = max(d_b, w_b / w_max)`.
    The dent centre moves as a block and the ring of bonds around it takes the offset difference, so ring cracks and revealed inner faces (section 6) appear around impacts; a bond that
    reaches `d >= 1` really breaks (section 2). Offsets are permanent.
  - Colliders stay at the rest pose (offsets <= `o_max`); a chunk that detaches takes its current visual pose as its body pose. Runs in the worker (`IntegrityCore` + `DentField`), which keeps the per-bond damage `d_b` and returns changed chunk poses (`o`, `th`) with its breaks. Parameters per material in the `dent` record; crack measure is `|o_j - o_i|` (opening + slip); contact point = hit chunk centre.
  - `deform: 'joints'` materials (metals, rubber, soft plastics) get real deformation (section 4b). Real deformation of brittle materials isn't worth it: strains before cracking are millimetres.
  - **Damage points (current, 2026-11-17)**: only the hit chunk moves. `D_i += J / jRef`, `f = 1 - exp(-D)`, per hit `df = f(D') - f(D)`: `o_i += oMax size_i df n` (n = incoming push dir, |o| <= oMax size), `th_i += tilt df a` with `a` a random unit axis per hit (seeded xorshift, |th| <= tilt rad). No neighbour falloff, no per-bond crack damage: dents never fracture (they did cascade on the wall demos). Render: when `dent.revealInner` (per material; false for steel/aluminium/rubber, skinned surfaces don't gap) a chunk whose static pose changes gets its bonds' inner faces revealed (main `syncRender`), since the gaps open without a break. Record: `dent { oMax, jRef, tilt, minForce, offset, rotate, revealInner }`; `offset`/`rotate` gate the positional/random-rotation terms (steel/aluminium/rubber: rotate off; note DentField only runs for `deform: 'offsets'`, joint-chain plates deform via 4b). 
  - `deform: 'none'` (glass): no dent; with `propagation > 0` it shatters instead.
- Candy-wrapper from linear blending only on strong twist; switch to dual quaternions if it shows.
- Tests: CSR prefix sums consistent, in-place shrink never overruns a slot, GPU and CPU skin agree for counts 1, 8, 32, partition of unity (sum w = 1), rest pose is identity, seam copies identical while intact and disjoint after a break, bent-sheet normals stay unit.

### 4b. Real bending with engine joints (`deform: 'joints'`; replaces the plastic-hinge/FK model)

Each chunk of a *bendable part* (steel/aluminium panel, post, rail, door; rubber and clay too, with `sigmaY = 0` meaning springs back and a low `sigmaY` meaning clay-like) is a dynamic Rapier body; neighbours are linked by a **compliant joint** instead of a rigid fixed joint.
- Why not a pure fixed joint: its error is the solver's residual (depends on mass, dt, iterations, joint frequency), not on a material stiffness, so
  error -> stress is not controllable and differs per chunk mass, and there is no motor target to carry a plastic set.
- **Joint (decided by the M4 spike, NOTES "M4 joint spike")**: Rapier JS has no 6-axis motors, and explicit springs are ~1e5x too soft under their stability cap
  `k <= m_eff omega_max^2` (`omega_max ~ 1/(2-4 dt)`). Use a **spherical joint (translation locked) with 3 implicit angular motors** to the rest orientation:
  torsion `GJ/L`, bending `EI1/L`, `EI2/L` (bond section values, section 1), scaled by ~1e-2 so the motor compliance dominates the solver error; 4 substeps, 16 iterations.
  Moments `M = k phi` are the load the bond carries in quasi-static equilibrium (~9% of statics in the spike), so the scaling only exaggerates elastic sag; yield and fracture moments stay right.
  Axial/shear are rigid and not measured (`N = V = 0`, or from equilibrium if needed); metals fail by bending first.
- Error `phi: (3,)` from body poses each step: `phi = log(q0^-1 q_a^-1 q_b)` (relative rotation vs rest `q0`, in the parent frame).
- Plasticity = rest-shape update on the motor targets (VIPER-style): `[0, 0, 0, Tq, M1, M2]` from `M = k phi` through the corner formulas of section 2b (von Mises at the worst corner);
  if `sigma > sigma_y`, `q0 <- q0 exp((1 - sigma_y/sigma) phi)`, then push the new targets to the motors. Each step, not per frame; hardening via `sigma_y + H eps_p`.
  **Rest-state bake**: in 2D parts (grids with loops) per-joint targets become mutually incompatible with the locked anchors (doubly curved dents need in-plane stretch),
  so the part stays strained and the solver jitters forever. Only parts with loops are baked (a tree has no incompatibility, and baking a loaded part drops its load and ratchets). When a part stops yielding for ~0.1 s, its current shape becomes the stress-free rest (all targets = phi, anchors moved together);
  residual elastic stress at that moment becomes permanent set. (Continuous anchor slip was tried and pumps energy; NOTES "Joint-chain perf".)
- **Carriers** (2026-11-17): a joint whose static side detaches into a rigid island moves onto that island body (side a; anchor/frame from the chunk's island-local pose `(t, r)`: `a = t + r (centroid - chunkPos)`, frame `r B`, kernel `q0 = r`; targets negated when the static side was b). The part loses that static anchor (free damping) but stays connected. Kernel `|dx|` stays exact but its components are in a basis rotated by `r` (only `|dx|` is used, `jointState.gap`); carriers are body slots with part -1, dead ones point at the static body.
- Fracture: plastic strain above `epsPlasticMax`, or filtered moment-based ratio > 1 -> remove the joint. The two sides are already separate bodies, so
  **no island labeling is needed for these parts**.
- Poses come for free (body transforms), colliders follow the bodies, and large bends are exact. Tier A (section 5) is automatic here; tier B adds shape change.
- Weaknesses: (1) stiff chains of iterative joints sag/stretch (more solver iterations, short chains, mass ratios <~10); (2) the part is awake while deforming and an awake 90-body plate costs ~10-15 ms/frame in Rapier: absolute body damping
  (2/s linear + angular while connected to an anchor) settles it in ~2 s; pieces cut off from every anchor drop to near-realistic damping (0 / 0.05) so debris flies freely and our rest test sleeps it (joint damping is relative and leaves whole-part modes ringing); (3) loads from the *static* structure (a slab resting on a steel beam) are not seen by the dynamic chain.
  Therefore scope: parts that mostly carry their own weight and impacts. For a structural beam, the external solver provides the interface
  bond forces as external forces on the chain's end bodies (`applyWrenches`), or the beam stays on the static solve with damage only (no large bend).
- Promotion: a `joints` element lives in the static graph until impacted (or scripted) or its solver ratio exceeds ~0.7 (M3b), then its chunks are promoted to dynamic bodies + joints
  (fixed body anchors for supported ends). Static `joints` chunks detached by integrity (cut loose together with non-joint neighbours) are promoted as an unanchored chain instead of a rigid island. Demotion when at rest for a while (~1 s, anchored, not yielding): bodies become static colliders at their current pose (a per-chunk static pose replaces the rest pose; islands carry per-chunk body-frame poses so later detaches don't pop), plastic targets/strain are kept per bond and restored on re-promotion.
- **Who runs it**: the joint-error/plasticity/fracture loop needs the body poses, so it runs where Rapier runs (main thread, or the physics worker if split out).
  Handshake with the integrity worker: on promotion the worker removes those chunks from the static solve and treats the interface chunks as supports (the chain carries its own load);
  on a joint break the main side sends the bond as a removal (same path as scripted cuts), on demotion no message is needed now (promotion never left the worker's graph; bonds stay intact) — the external solver (M3b) will need the frozen poses then, so the worker's graph, weights and culling stay in sync.
- **Self-collision**: each chunk of a chain is its own body with an overlapping hull (section 5). `setContactsEnabled(false)` only covers directly jointed pairs, and collision groups are 16 bits,
  so use `partId mod P` as the group (parts sharing an id lose mutual collision, rare when far apart), or a `filterContactPair` hook comparing part ids if that proves wrong.
  Implemented: membership = the part bit only (a shared debris bit would re-enable self-contacts), filter = all but the part bit; a body that loses its last joint returns to debris groups.
  Group bit budget (16 membership + 16 filter bits) is shared: e.g. bit 0 structure, bit 1 debris, bit 2 grace (Rapier notes), the remaining bits for `P` part ids.

### 5. Fast convex hull refit (materials with `hull: 'refit'` only; the rest never refit)

Never run a hull algorithm per update. **Fixed-topology hull**:
1. **Hull input = the mesh vertices the cell influences**, not a separate proxy set. Per chunk keep a vertex list `V_c = { v : w_c(v) > 0 }`
   built from the skin weights of section 4 (any influence counts). The hull is bigger than the cell and overlaps its neighbours' hulls; that is accepted.
   Overlap handling (verified in M1, see Rapier notes below): colliders of one body never collide with each other, and fixed-fixed pairs
   generate no contacts, so the intact structure and each island are safe. Cross-body overlap right after a split is the risk.
   Because seam vertices are duplicated per side and weights drop across a broken bond, `V_c` never contains vertices from the far side of a crack and
   already includes the new cut face: the collider closes at the fracture with no extra logic. On a break, rebuild `V_c` only for the chunks whose
   vertices were re-weighted.
2. Online: skin the vertices of `V_c` (section 4 formula, using the existing weights), then cut the candidate list to <= ~26 points with directional extremes
   (the support point of `V_c` along ~26 fixed directions in the chunk's rest frame; the directions' winning vertex ids are cached and re-evaluated in O(|V_c|)).
   The hull is then computed over <= 26 points, and its face indices are cached (step 3 revalidates them). The topology is recomputed only when `V_c` changes (after a break) or the check fails.
3. Convexity check, O(F V) ~ 400 ops: every vertex must be behind every face plane (tolerance eps). Pass -> hand points + the *same indices*
   to Rapier (`ColliderDesc.convexMesh(vertices, indices)` / `collider.setShape`; no quickhull). Fail -> fall back to a real hull on those points (rare).
4. Dirty policy: refit only when a chunk's pose changed more than ~2 deg / a fraction of its size since the last fit, throttled to a per-frame budget (e.g. 16 chunks), never for sleeping debris.
Tier A (default): no shape change; the collider keeps its rest hull and just follows its body (joint chains) or stays at the rest pose (`offsets`).
Tier B (this section, `hull: 'refit'`) is a toggle. Verify in the Rapier build: `convexMesh` really skips quickhull, `setShape` cost, f32 precision at the boundary.
Tests: refit hull equals a reference quickhull to 1e-5 for mild bends; check detects a folded (non-convex) case; allocation-free.

### 6. Render culling: inner cells and inner materials

Rule: **draw only what can be seen**. Intact structure = its outer surface only; fracture (inner) material appears exactly where a bond opens or breaks.

- **Generation tags** (in `ChunkModel`): every chunk face is `outer` (lies on the source surface, outer material) or `inner` (a cut face, inner material) and an inner
  face stores its `bondId` (the neighbour it touches). A chunk with no outer face is an *interior cell* (never touched the surface in the fracture pass).
- **Visibility**: outer faces always drawn. An inner face is drawn iff its bond is *revealed*: broken, or damage `d > d_show` (~0.05-0.1, so the visual crack gap
  shows its interior as it opens, section 4). Interior cells therefore contribute **zero** triangles until a neighbour breaks, and then only the faces toward the broken bond(s).
  Bond-level is the exact form of "a broken cell or a broken neighbour": revealing all inner faces of the cell is the simpler fallback (more overdraw, same look).
  A cell destroyed or despawned reveals through its bonds first (the bond break is always emitted before the cell goes away), so neighbours never show holes.
- **Monotonic and append-only**: once revealed, a face stays visible (even on debris). So two meshes share one vertex buffer and one pose texture:
  1. outer material: static index buffer, built once;
  2. inner material: preallocated index buffer sorted so each bond's two cut faces (one per side) form a contiguous range; on reveal, copy that range into
     the next free slot, bump `drawRange`, upload only the appended slice. No removal, no rebuild.
  Despawned chunks are hidden through the pose texture (a "hidden" flag: the shader collapses their vertices to a degenerate point), since their indices stay in both buffers.
  Interior-cell vertices stay in the CPU arrays (hull input `V_c`, weights, re-weighting after breaks need them) but are in no index buffer, so the GPU never skins them.
- Skinned (`joints`) parts: seam cap faces are `inner`, hidden while intact (welded, identical weights = no gap) and revealed on joint/bond break, as for concrete.
- Gains: a solid wall of N cells has ~N^(2/3) surface cells, so draw/skinning cost scales with the surface, not the volume.
- Tests: initial index count = outer triangles only (interior cells 0); a break appends exactly that bond's two faces, idempotent; `drawRange` monotonic;
  revealed set after random break sequences equals a brute-force recompute ("faces whose bond is broken or d > d_show").

### 7. Execution model: stable FPS first (async, threads, WASM)

Principle: the frame never waits on destruction work. Everything below is a *latency* trade, not a correctness one.

**Frame budget rules**
- Main thread per frame does only bounded work: apply results, upload dirty GPU slices, a capped number of Rapier mutations. Caps are GUI-tunable:
  collider create/remove <= N per frame (queue the rest; islands appear a few frames later, tier 0 results are applied first), hull refits <= M, `drawRange`/weight uploads <= bytes.
- All heavy producers are **budgeted queues** (a time slice via `performance.now()`, or a fixed iteration count when determinism is wanted): structure-solver iterations, re-weighting after a break,
  `V_c` rebuilds, Voronoi generation.
- Results carry a **graph version**. A result computed against version `v` is applied only to bonds/chunks still alive; breaks always win over stale poses/ratios. Commands to the worker are ordered (sequence numbers).
- Backpressure: if the worker is behind, coalesce loads (sum impacts per chunk per tick) instead of queueing each one.

**Threads (possible from day one, off by default where it adds nothing)**
- *Integrity worker* (M3): structure graph + labeling now; external structure solver and fake-deformation offsets later. Talks via `postMessage` with transferable double buffers.
- *Geometry worker* (optional): weights/`V_c` rebuilds, hull candidate selection, Voronoi/glTF import. Same command/result protocol, so it is a second instance of one `Worker` wrapper.
- *Physics worker* (later, if Rapier + step cost is high): Rapier runs in its own worker, main thread only renders from posted pose buffers. The `PhysicsBackend` interface (top of this plan) makes this a swap; commands (create island, remove collider) become messages.
- Upgrade path to `SharedArrayBuffer` (chunk poses, damage, ratios as shared typed arrays with an atomic double-buffer index): needs cross-origin isolation headers
  (COOP/COEP, set in the Vite dev server and production host). Hide behind a small `SharedState` abstraction so the first version uses plain transfers.
- Every kernel is a pure function over typed arrays (no classes in hot loops, no allocation per call, SoA layout; matches the repo conventions), so it runs unchanged in main, a worker or WASM.

**Faster code: AssemblyScript / Porffor / Javy (assessment)**
- **AssemblyScript (AS): the only one worth planning for.** TypeScript-like syntax compiled to WASM, Float64 and SIMD (`v128`) available, typed-array style memory, small output. Targets: hot solver kernels (if the external solver is TS),
  chunk-pose/skin CPU kernels, hull refit (directional extremes + convexity check), Voronoi clipping. Caveats: it is a TS *subset* (no closures over locals, no structural typing, limited generics), SIMD f64 is only 2 lanes
  (so the f64 solver gains are modest; do not drop to f32, the Float64 lesson applies to stiff solves), and threading with shared memory is weakly supported (use one WASM instance per worker instead).
  Honest expectation: 1.2-3x over well-written V8 typed-array loops, more only for SIMD-friendly kernels. **Measure the TS version first** (repo rule); port only kernels that profile hot.
- **Porffor: no for now.** Ahead-of-time JS/TS to WASM, but experimental and incomplete; fine to watch, not to depend on for a game's core loop.
- **Javy: no.** It embeds a JS engine (QuickJS) inside WASM for sandboxing/portability, so it runs *slower* than V8; it solves a different problem.
- Fallback if AS limits bite: Rust (or C) to WASM for the same kernels; the boundary is identical.
- **How we keep the option cheap**: kernels live in `kernels/` with a TS reference and, when needed, an AS build behind the same signature `(memory views, offsets, counts) -> void`.
  One differential test runs both implementations on random inputs (tolerance 1e-9 in f64) and a micro-benchmark records the speed ratio in the notes, so a port only lands with a measured gain.

### Rapier overlap notes (verified in M1, `tests/rapier.test.ts`, NOTES "Rapier API facts")
- Same-body colliders: no contacts between them (compound is safe). Fixed-vs-fixed: no contacts by default.
- Jointed bodies: `joint.setContactsEnabled(false)` for the rebar-dangling case and directly jointed chain pairs (4b).
- Gap: a freshly detached island overlaps the remaining structure and its sibling islands at the crack. Active contacts would push out violently. Plan: put new islands in a temporary collision group that ignores their former neighbours,
  and restore the normal group once separated (cheap AABB/distance check per island, or after a fixed time). Collision groups are per collider,
  not per pair; if a pair rule is needed use physics hooks (`filterContactPair`), which cost a JS call per pair, so keep that as the fallback.
- Chunks of a debris island that later re-fractures overlap too; the same grace group applies.

### Milestones

**M1 Skeleton.** `ChunkModel`, a box/brick-grid generator (bonds from face overlap), Rapier harness with the backend interface,
the `Material` table with presets (section 2b), project setup (above), debug render (chunk colors, bond lines intact/broken; ratio colouring returns in M3b), face tags + surface flag, and the culled two-material render (section 6) from the start, since it shapes the index buffers. *No Voronoi needed to proceed.*
**M2 Bond solver** — *removed 2026-11-14 (replaced by an external solver, section 2)*. Was: (headless, written as pure typed-array kernels in `kernels/`, section 7, with a micro-benchmark for the AS decision): axial bar vs `FL/(EA)`; cantilever vs Euler-Bernoulli; free island equilibrium; warm-start speed-up; local effect of a break; no per-solve allocation. Measure PCG iterations at ~1000 chunks.
**M3 Core loop.** Worker + tier 0, component labeling, island handoff to Rapier. Done when removing a wall drops the dependents instantly. (Tier 1 stress/damage, failure ratios, impact loads and the material matrix test were removed with the solver; they come back with the external solver.)
**M3b External structure solver integration** (when available): wire it into `IntegrityCore` per the section 2 contract; restore impact loads, overload-driven breaks, the material matrix test, and the stress/damage debug views.
**M4 Skinning + deformation** (sections 4, 4b): fake offsets for `offsets` materials (impact dents, ring cracks); `joints` materials (steel panels/posts) as joint chains with the promotion handshake (impact/scripted until M3b), plasticity by rest-target update, skinned from body poses. Tests: cantilever sag vs Euler-Bernoulli (expect some error; record it), permanent set after unload, fracture removes the joint and separates bodies, stability over 10 s.
**M5 Hull refit** (section 5) with the tier A/B toggle and budget GUI.
**M6 Voronoi (non-priority):** per site clip the element's convex volume by bisector planes of its k nearest sites; the clipped polygon on each plane is the bond face, so adjacency/area/moments come free. Graded density, Poisson-disk sites. Concave buildings are built from convex elements (columns, slabs, panels around openings), each fractured in its own volume, bonded by coplanar face overlap. Blender route: any splitter -> glTF, one node per chunk + `extras`; importer builds bonds by AABB overlap + coplanar opposite-normal face clipping. Tests: volumes sum to the source (1e-9), symmetric adjacency.
**M7 Game scene + perf** (frame-budget caps and queues from section 7, worker/SAB upgrades and AS ports only where profiling justifies them): level, wrecking ball/explosive, GUI, debris cap + despawn/merge of tiny far pieces, profiling before optimizing.

### Risks
1. External structure solver integration: data contract (section 2), cost per tick under the frame budget, and how its outputs feed 4/4b (promotion, offsets).
2. Joint-chain plasticity (4b) is the least proven part: the rest-target update and the promotion/demotion handshake with the integrity worker. Joint stiffness/stability was validated on a cantilever in the M4 spike (NOTES); exaggerated sag (~200x Euler-Bernoulli) is the accepted cost.
   Related: three deformation sources (static solve, fake offsets, joint chains) must agree on the render pose at promotion/demotion; test for pops.
3. (If the external solver is a small-displacement linear solve) no P-delta: slender columns won't buckle by themselves; add an Euler-load check per column if the game needs it.
4. Rapier JS API details: re-parenting (absent), contact-force events, per-axis motors (spherical only) verified in M1/M4 (NOTES); still open: does `convexMesh` with indices skip quickhull, `setShape` cost (M5).
5. Overlapping hulls (`w > 0` influence): cross-body overlap after splits (grace group, Rapier notes) and chain self-collision (4b) are the practical hazards; solver is Float64 but Rapier/GPU are Float32, so keep poses relative to the island/structure origin to limit precision loss on large levels.
6. Collider recreation on detach (hundreds of colliders in one frame): budget/spread across frames if it spikes.

# Progress (formerly PROGRESS.md)

### M1 Skeleton
- [x] Project setup + folder layout + `NOTES.md`
- [x] `ChunkModel` packed arrays (chunks, bonds, face tags, surface flag) — Design §1
- [x] Material table + presets + load-time validation — Design §2b
- [x] Box/brick-grid generator, bonds from face overlap
- [x] `PhysicsBackend` interface + Rapier impl (joints: fixed/ball only; spring targets deferred to M4)
- [x] Verify Rapier API (results in NOTES.md). Still open: does `convexMesh` skip quickhull internally, `setShape` cost (→ M5)
- [x] Debug render (untested in browser): chunk colours, bond lines coloured by ratio
- [x] Culled two-material render (untested in browser) (outer static IB, inner append-only IB, `drawRange`) — Design §6
- [x] Tests: generator adjacency, culling (initial = outer only, append idempotent, monotonic, brute-force match)

### M2 Bond solver (headless kernels) — Design §2, §7 — REMOVED 2026-11-14 (code in git history before this date)
- [x] Bond stiffness `K_b`, CSR 6x6 assembly
- [x] Block-Jacobi PCG, warm start, time-sliced
- [x] Tests: axial bar, cantilever vs Euler-Bernoulli, free-island equilibrium, warm-start speed-up, local break effect, no per-solve allocation
- [x] Benchmark: PCG iterations @ ~1000 chunks (record in NOTES.md)

### M3 Core loop (brittle collapse)
- [x] Integrity worker + messaging (double buffers, versions) — not yet run in a browser
- [x] Tier 0 connectivity (now `StructureGraph`). Tier 1 stress/damage + failure ratios were done, removed 2026-11-14
- [ ] ~~Material matrix test~~ removed with the solver; returns in M3b
- [x] Component labeling + island handoff to Rapier (mass/inertia, grace group) — headless + Rapier test; not in the app yet
- [ ] ~~Impact loads from contact events~~ removed with the solver (backend `drainImpactEvents` + `dir` sign rule kept); returns in M3b
- [x] Done-when: wall removal drops dependents (headless test). Overload-driven cracking removed with the solver.

### M3b External structure solver integration — Design §2 contract
- [ ] Plug the external solver into `IntegrityCore` (reads `StructureGraph` + materials + masses + loads, returns bonds to break)
- [ ] Restore impact loads, material matrix test, stress/damage debug views (chunk/bond heatmaps, solver stats)

### M4 Skinning + deformation — Design §4, §4b
- [x] Variable-length weights (CSR, topology-aware, in-place reweight on break) + CPU skin reference (`model/skinWeights.ts`, `kernels/skin.ts`, `tests/skin.test.ts`)
- [x] Pose texture + TSL skin (`render/skinMaterial.ts`), wired into `chunkMesh`/`main.ts`; islands posed via `writeIslandPoses` (CPU vertex placement removed). Browser-verified (concrete wall, new 'steel plate' scene: cut + knock-off, clean cut edge)
- [x] Automated GPU vs CPU skin agreement for counts 1, 8, 32: `npm run test:gpu` (WebGPU + WebGL2 fallback, max err ~1.3e-6)
- [x] Fake offsets (dents; ring cracks removed 2026-11-17, now damage points on the hit chunk, `dent.offset/rotate/revealInner` flags; browser re-check pending): `integrity/DentField.ts` in the worker, `Material.dent` record, impacts in TickRequest, poses in TickReply -> static chunk poses (`tests/dent.test.ts`). Browser-verified on 'concrete wall'. Elastic `u_el` term waits for M3b
- [x] Spike (cantilever chain, 10 s): (a) explicit springs, (b) fixed joints, (c) motors — found in Rapier itself (spherical joint angular motors), no Jolt needed. Chosen (c); numbers in NOTES "M4 joint spike". Backend: `createSpringJoint`, `setSpringTarget`, `readBodyStates`, `applyWrenches`.
- [x] Joint chains (`sim/JointChains.ts`, `kernels/jointPlasticity.ts`): promotion on impact (>= 2000 N) or script, one body per chunk, spring joints (anchored to the static body at the part's edge), plasticity by motor-target update, fracture -> bond removal to integrity; `IslandHandoff.detach` skips chunks already on bodies. Browser-verified on 'steel plate' (strength x0.01: smooth plastic bend)
- [x] Joint-chain perf: body damping + own rest test -> sleep (hit plate asleep in ~3 s, 0.4 ms/frame after), part collision groups, rest-state bake fixing endless jitter of plastically dented plates (`tests/chainPerf.test.ts`, NOTES "Joint-chain perf")
- [x] Demotion (at rest -> static colliders at the deformed pose, plastic set kept, re-promotion resumes without pop; no worker message needed until M3b). Browser-verified on 'steel plate' (hit -> 90 bodies demoted after settling, re-hit re-promotes, bends further, demotes again)
- [x] Tests (`tests/jointChains.test.ts`): kernel yield return, steel stable 10 s (sag 8.2 mm), permanent set after unload (34.5 of 35.1 cm kept; elastic springs back to -1.4 mm), fracture removes joint + bond, chain falls, sag vs hinge-chain/Euler-Bernoulli (sim 5.9 mm = 1.36x discrete hinge at k x1e-2; raw EB 0.038 mm; NOTES)

### M5 Hull refit — Design §5
### M6 Voronoi — M6
### M7 Game scene + perf — Design §7, M7
### Decisions / deviations log
- 2026-11-17: Joints to a static chunk that detaches are re-bound to its rigid island (was: dropped, the chain fell off and overlapped the island). Users want curtains/rails to stay on their falling support.
- 2026-11-17: Joint parts cut loose while static become free joint chains (was: rigid island, stayed stiff). Settle damping only while anchored; disconnected = near-realistic damping.
- 2026-11-15: Projectile breaks use a KE >= fracture-energy test (dt-independent), not contact force.
- 2026-11-16: Collision breaks apply to every contact pair (relative energy 1/2 mu v_n^2), correcting velocities by handing back the absorbed impulse after the step, not by contact hooks (keeps Havok swap simple).
<!-- YYYY-MM-DD: decision — reason -->
- 2026-11-17: Dents = per-chunk damage points on the hit chunk only (offset along hit dir, random tilt axis), no falloff and no crack breaks: dent cracks caused unwanted fractures on wall demos. `dent.radius/wMax` removed.
- 2026-11-15: Fake dents: `Material.dent` record (oMax/jRef/radius/tilt/wMax/minForce); saturation on accumulated impulse per hit chunk (contact split into substep events); crack measure |o_a - o_b| (opening + slip); contact point = chunk centre; tilt scaled by s/(R h)^2 (the design gave no impulse scaling).
- 2026-11-15: Demotion: islands get per-chunk local poses + chunks a static pose (`setPos/setQ`), `IslandHandoff.attach`; no `demote` message (graph never saw promotion). Rest-state bake restricted to parts with loops (tree parts ratcheted under gravity). Re-promotion restores stored targets instead of baking.
- 2026-11-15: Joint chains: absolute body damping + own sleep (Rapier cost is the iterations; settling is the lever), part bit as sole membership, rest-state bake after each yield episode (loop incompatibility in 2D parts). Costs: steel sag 5.9 vs 8.2 mm, residual elastic stress baked into permanent set. Kept 16 iters x 4 substeps.
- 2026-11-15: Island re-fracture supported (supersedes "island re-fracture not supported" below): integrity processes bonds of floating parts too; TickReply gains `floatChunks/floatOffset`.
- 2026-11-15: Joint-chain motor targets are additive per-axis rotation vectors (bond frame), matching Rapier's per-axis motor angles; fine for moderate bends, revisit for crumpling. Promotion = one single-chunk island per chunk (reuses handoff, render and grace logic). App defaults now 4 substeps / 16 iterations (chains need them).
- 2026-11-15: Skin data in data textures, not storage buffers — one path for WebGPU and the WebGL2 fallback. Whole-texture uploads for now (tiny).
- 2026-11-15: Design consistency pass — §4b rewritten to the spike result (spherical joint + 3 angular motors, error `phi: (3,)`, rotational plasticity only, stiffness scaled ~1e-2 instead of the explicit-spring cap); corner-stress formulas moved into §2b (§2 no longer has them); backend method list matches `PhysicsBackend.ts`; M1-verified Rapier facts marked verified; not-yet-wired parts (impact loads, worker poses, island re-fracture, ratio-based promotion) flagged.
- 2026-11-14: Removed the in-house structure solver (bond stiffness, PCG, inertial relief, failure ratios, damage, StressTier/BondSolver, impact loads, stress heatmaps, solver GUI/stats, solver/integrity/materialMatrix/impact tests) — results/feel not as wanted; an external solver will take the structure graph (mass + material data) as input. Kept: ChunkModel, materials, `StructureGraph` (alive + adjacency + tier 0), worker boundary, island handoff, joint spike. Supersedes the solver/impact entries below.
- 2026-11-14: 4b joints = spherical impulse joint + implicit angular motors (stiffness x1e-2, 4 substeps, 16 iters) — explicit springs ~1e5x too soft under the stability cap; fixed joints give uncalibrated stress and no plastic target. Supersedes "explicit forces/torques" below.
- 2026-11-14: Impact peaks held >= 0.05 s and until a converged solve (was: one request) — single-step loads were filtered away; PCG continues across requests when loads are unchanged (supersedes "always restart") — restarts stalled convergence after breaks.
- 2026-11-14: Worker always restarts PCG on the latest loads each request (warm start keeps progress) so long solves never hide new impacts; scripted removals are queued and applied by the next request (one tick latency). Island re-fracture not supported yet (handoff skips components already debris).
- 2026-11-14: Moving islands rendered by CPU rigid placement into the shared vertex buffer (debug path; replaced by M4 pose texture). Tier 1 runs only on converged solves (main thread v1).
- 2026-11-14: New `src/sim/` layer glues integrity + physics (both allowed there); grace window 0.1 s (longer lets islands sink into ledges). Material-matrix wall case asserts "a highly loaded bond fails", not the probe argmax (damage redistributes load in indeterminate structures).
- 2026-11-14: Warm-start test uses tier-1 tolerance 1e-4 (no gain at 1e-8); after large breaks warm can lose to cold — decide restart policy in M3.
- 2026-11-14: 4b joint springs use explicit forces/torques (Rapier JS GenericImpulseJoint has no per-axis motors).
- 2026-11-14: Code under `src/` (design folders as `src/model` etc.); vitest for tests; tsc 7.
- 2026-11-14: Added `createGround`/`createSphere` to PhysicsBackend for demo/projectiles.
- 2026-11-14: Created agent docs (since merged into CLAUDE.md + NOTES.md); no code yet.

### Session log
- 2026-11-17: Fixed rubber strips sinking through the ground after a while (zombie sleep, NOTES gotcha): `RapierBackend.step` wakes sleeping bodies in the active set. New `tests/chainGroundSleep.test.ts` (fails without the fix). 95 tests pass, typecheck OK; not browser-checked.
- 2026-11-17: Joint chains stay attached to a detached rigid support (carrier re-bind, Design §4b): rubber curtain strips hang on the falling lintel instead of dropping off/glitching. `tests/chainCarrier.test.ts`; `chainDetach` now expects the re-bound anchor. 94 tests pass, typecheck + build OK; not browser-checked.
- 2026-11-17: UI pass (`src/ui/`: HUD, hint bar, themed/regrouped GUI, P/R keys); pre/post-fracture damping (anchored = settle damping, disconnected = free 0/0.05); detached static `joints` chunks promote to a free chain (`tests/chainDetach.test.ts`; demoted-cut test now expects a chain); fix: free damping per unanchored fragment, not only when the whole part loses its anchors. 92 tests pass, typecheck + build OK; screenshot-checked UI, damping/chain change not browser-checked.
- 2026-11-17: 7 joint demo scenes (steel cantilever/beam/truss bridge/frame/tower/pole, rubber curtain) + `SCENE_STRENGTH` per-scene strength defaults (`main.ts` `selectScene`, exposed on `window.demo`), `tests/jointScenes.test.ts`; browser-checked (all load, promote, demote; pole/curtain bend visibly). 90 tests pass, typecheck + build OK.
- 2026-11-16: Joint-chain promotion by collision no longer deflects the hitter: `CollisionBreaker.passThrough` undoes the bounce off the static part, momentum goes into the promoted plate (ball decelerates over ~30 substeps, plate springs back). 75 tests pass, typecheck + build OK; not browser-checked.
- 2026-11-16: Fixed collision-break explosions / ball sliding (NOTES gotcha): immediate `splitChunk`, pre-step velocities from the backend, per-pair energy budget, CoM hand-back measured at apply time; `ImpactEvent.point` and `applyImpulseAtPoint` replaced by `applyImpulse`. New tests/collisionStress.test.ts (12 scenes, delayed integrity). 74 tests pass, typecheck + build OK; not browser-checked.
- 2026-11-16: Option A generalized to every contact pair: `sim/CollisionBreaker.ts` replaces ImpactBreaker (debris/ground/island/ball pairs break chunks, pair keeps momentum minus fracture energy); backend `bodyMass`, `applyImpulseAtPoint`, `ImpactEvent.forceVec/point`; GUI dents toggle, break min speed, `collisionBreaks` stat. 62 tests pass, typecheck + build OK; not browser-checked.
- 2026-11-15: Impact breaking option A (`sim/ImpactBreaker.ts`): balls cut the static chunks they hit and keep momentum minus fracture energy; `PhysicsBackend.setLinvel`, `ImpactEvent.bodyA/B`; GUI 'ball break energy x'. 61 tests pass, typecheck + build OK; not browser-checked. Option B (pre-step shapecast, no bounce frame) if the 1-step hitch shows.
<!-- YYYY-MM-DD: what was done, what's next -->
- 2026-11-15: GPU skin harness (`npm run test:gpu`, playwright-core devDep, `skincheck.html`): WebGPU and WebGL2 match the CPU kernel to ~1e-6 (counts 1/8/32, positions + normals). WebGL2 fallback now verified. 58 tests, typecheck + build OK.
- 2026-11-15: M4 chain sag vs hinge/EB test recorded (sim 1.36x softer than ideal hinge chain); 58 tests pass, typecheck OK.
- 2026-11-17: DentField rewritten (damage points, hit chunk only, no bond breaks); inner faces revealed around dented chunks, per-material `dent.revealInner/offset/rotate`; merged `damage-dents` into master. 76 tests pass, typecheck OK; not browser-checked.
- 2026-11-15: M4 fake dents (DentField, request/reply plumbing, sim static poses), 3 tests; demotion browser-verified; 57 tests pass, typecheck + build OK.
- 2026-11-15: M4 demotion (JointChains.demoteRestingParts, IslandHandoff.attach/local poses, main.ts static pose upload), bake ratchet fix for tree parts, 2 new tests; 54 pass, typecheck + build OK. Not browser-checked.
- 2026-11-15: Joint-chain perf + stability: profiled (Rapier dominates), damping + sleep, part collision groups, rest-state bake; backend `setBodyDamping`, `sleepBodies`, `setJointAnchors` (used by the bake). 52 tests pass, typecheck + build OK; browser: ball-hit soft plate curls, settles, all asleep, simTickMs 0.
- 2026-11-15: Fix: breaks inside detached pieces were dropped by `StructureGraph` (never reached `brokenLog` -> no skin reweight -> stretching; chain joint breaks after the chain fell too). Graph now processes floating bonds and reports their components; `IslandHandoff.refracture` splits islands. Tests: graph float split, handoff no-pop split; 51 pass, build OK; browser: steel plate top half cut then split, 0 vertices blend across the halves.
- 2026-11-15: M4 joint chains + plasticity + fracture, impact promotion in `DestructionSim.tick`, `sim.alive`; 50 tests pass, typecheck + build OK; browser-checked plate bend. Perf open (see Next).
- 2026-11-15: GPU skin: pose RGBA32F + entries R32UI data textures (textureLoad, so the WebGL2 fallback should work too; untested), `skinRange` uvec2 attribute, positionNode assigns normalLocal; meshes frustumCulled=false; 'steel plate' scene. 46 tests, typecheck + build OK, browser-checked via playwright.
- 2026-11-15: M4 skin weights (CSR, unorm16 summing exactly to 65535, topology BFS inside support, alloc-free `reweightBroken` with dirty list) + CPU skin kernel; 5 new tests, 46 pass, typecheck OK. GPU path not started.
- 2026-11-14: Removed structure solver (see Decisions). New `integrity/StructureGraph.ts`, `kernels/bondAdjacency.ts`; IntegrityCore/DestructionSim now removal-only (request sent only when removals are queued); bond lines = intact/broken. 41 tests pass, typecheck + build OK; not browser-checked.
- 2026-11-14: M4 joint spike: `kernels/jointSpring.ts` (error/wrench/cap), backend spring joints + body states; `tests/jointSpring.test.ts` (6 tests); 93 tests pass, typecheck OK. Note: `integrityCore` "continues PCG" test failed once (timing-based, `budgetMs: 0.001`), passed 3 reruns.
- 2026-11-14: Debug UI: structure-sim stats (TickReply.stats + damage), chunk/bond heatmaps, sync/async fracture toggle, sim params (Hz, substeps, budget, tol, hold, gravity, Rapier iters), material multipliers. 87 tests pass, build OK; not browser-verified.
- 2026-11-14: Demo made to work (browser-verified): impact hold + PCG continuation fixes, cursor-aimed throws, shift+click chunk removal, 7 new scenes (slab on columns, tower, bridge, cantilever balcony, house, brick gate, ice pillars); 85 tests pass, build OK.
- 2026-11-14: Read the plan, set up agent docs.
- 2026-11-14: M1 implemented: model, materials, generators, culling, Rapier backend + facts tests (20 passing), debug app. Build OK. Not yet viewed in a browser.
- 2026-11-14: M3 worker: IntegrityCore (message handler), Inline/WorkerIntegrity clients, integrity.worker.ts, DestructionSim one-request-in-flight; 83 tests pass, build emits worker chunk.
- 2026-11-14: M3 impacts (ImpactLoads), DestructionSim main-thread loop, CPU island placement (render/islandPose), main.ts wired; 80 tests pass, build OK.
- 2026-11-14: M3 material matrix (36 cases), islandMass (principal inertia), IslandHandoff (sim/), IslandDesc.inertiaFrame; 74 tests pass, build OK.
- 2026-11-14: M3 tier 0/1: failureRatio + damage kernels, Connectivity, StressTier, BondSolver.active mask; 7 tests (34 total) pass.
- 2026-11-14: M2 implemented: bond stiffness, block CSR, PCG, inertial relief, BondSolver; 7 solver tests (27 total) pass; benchmark in NOTES.
