# NOTES

## Commands
- `npm run dev` (Vite, debug app at /), `npm run typecheck` (tsc 7, noEmit), `npm test` (vitest, Node), `npm run build`.

## Architecture map (`src/`)
- `model/` headless data: `ChunkModel.ts` (packed SoA format, flags, OUTER_FACE), `Material.ts` (records, presets, validation, derived interface materials), `boxGenerator.ts` (boxes -> ChunkModel: bonds from face overlap, face splitting + tags), `generators.ts` (grid, running-bond brick wall).
- `kernels/` pure typed-array math: `bondAdjacency.ts` (chunk -> bond CSR), `jointSpring.ts` (below).
- `integrity/` `StructureGraph.ts` (alive per bond, attached per chunk, adjacency, `removeBonds` -> tier 0; seam for the external structure solver), `connectivity.ts` (tier 0 early-exit BFS -> detached components), `loads.ts` (chunk masses, own rotational inertia).
- `physics/` `PhysicsBackend.ts` (interface, group bits, packGroups), `RapierBackend.ts` (only Rapier importer), `fromModel.ts` (Float64 -> Float32 hull conversion).
- `kernels/jointSpring.ts` joint error `e = [dx, phi]` in the bond frame from body states (13 doubles/body), spring wrench, stiffness cap (PLAN §4b spike).
- `integrity/islandMass.ts` mass/COM/principal inertia (Jacobi) from rest cell volumes.
- `integrity/IntegrityCore.ts` message handler (TickRequest{remove} -> TickReply: broken, detached, version); `IntegrityClient.ts` (`InlineIntegrity`, `WorkerIntegrity`); `integrity.worker.ts` entry.
- `sim/` glue between integrity and physics: `IslandHandoff.ts` (static body + per-chunk colliders, detach components -> dynamic islands, grace -> debris groups), `DestructionSim.ts` (per-physics-step `tick`: poll reply -> breaks/handoff, send queued removals when none in flight; `brokenLog`, `isStatic` for renderers).
- `render/islandPose.ts` CPU rigid placement of island chunk verts (debug until M4).
- `main.ts` app: physics.step -> sim.tick per fixed step; `syncRender` reveals broken faces, places islands, hides broken bond lines.
- `render/` `cullBuffers.ts` (headless outer/inner index buffers, append-only reveal), `chunkMesh.ts` (two meshes sharing one vertex buffer), `bondLines.ts` (debug lines, intact green / broken hidden).
- `scenes.ts` 11 demo scenes (box lists, mixed materials, `carve` for openings) + per-scene camera; `main.ts` debug app (GUI: scene, visibility, break random bonds, stats; click/space = ball aimed at cursor with gravity lead, shift+click = cut all bonds of the static chunk under the cursor). GUI folders: debug view (outer/inner faces, bond lines), sim (sync = `InlineIntegrity`, pause/step, time scale, physics Hz, substeps; gravity/impact threshold/Rapier iters reload), material multipliers (`scaleMaterials`, reload; only density matters until the external solver), stats (version, replies, latency). `window.demo` exposes `cur` (model/sim/physics), `load`, `throwBall` for console/automated checks.
- `tests/` model, culling, Rapier facts, graph (tier 0), integrityCore (boundary + slow client), handoff, islandMass/Pose, jointSpring.

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
- Solver floor (sag of fixed joints): 0.50 m (4 it, 1 sub), 5.1 cm (4 it, 4 sub), 4.5 cm (16 it, 1 sub), 3.2 mm (16 it, 4 sub). Chains need substeps + iterations.
- (a) explicit springs (`kernels/jointSpring.ts`): kernel exact on short chains (sag within 1% of capped statics) but the stability cap `k <= m omega^2` makes steel ~1e5x too soft (predicted 6 m sag at 4 substeps), long chains barely damp (joint-relative damping leaves low modes undamped). A translational spring at lever l also needs `k l^2 <= I omega^2`, else bodies spin up. Kept only as the error/moment readout (k = motor stiffness, c = 0).
- (b) fixed joints: stiffest at few iterations, but error -> stress is uncalibrated (PLAN §4b) and there is no motor to move a plastic rest.
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

## Gotchas
- Rapier `EventQueue(true)` auto-drains on every `world.step`: with substeps, call `sim.tick` after each substep or impacts are lost.
- Bond lines use additive blending: black = hidden (broken / below threshold).
- Browser check without a user: playwright-core + system Chrome (`--enable-unsafe-webgpu`, headed, off-screen window) drives `npm run dev`; keep the harness outside the repo.
- tsconfig lib has no `WebWorker` (it breaks DOM event typing). Don't use `/// <reference lib="webworker" />` either (applies program-wide); type the worker scope minimally as in `integrity.worker.ts`.
- `ChunkModel` and `MaterialTable` (Map, Infinity) structured-clone fine; worker init clones them once.
- WebGPU rejects zero-size buffers: inner index buffer capacity kept >= 3.
- Bundle is ~5 MB (three/webgpu + Rapier base64 wasm); fine for dev, code-split later (M7).
