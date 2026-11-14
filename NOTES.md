# NOTES

## Commands
- `npm run dev` (Vite, debug app at /), `npm run typecheck` (tsc 7, noEmit), `npm test` (vitest, Node), `npm run build`.

## Architecture map (`src/`)
- `model/` headless data: `ChunkModel.ts` (packed SoA format, flags, OUTER_FACE), `Material.ts` (records, presets, validation, derived interface materials), `boxGenerator.ts` (boxes -> ChunkModel: bonds from face overlap, face splitting + tags), `generators.ts` (grid, running-bond brick wall).
- `kernels/` pure typed-array math: `bondStiffness.ts` (kd + K_b blocks ii/ij/jj, bond forces), `blockCsr.ts` (6x6 block CSR build/assemble/matvec), `pcg.ts` (block-Jacobi PCG, begin/iterate slices), `inertialRelief.ts` (free-island load balancing), `failureRatio.ts` (corner stresses -> ratio, `MaterialViews`), `damage.ts` (low-pass, damage, removal, propagation; chunk->bond adjacency CSR).
- `integrity/` `BondSolver.ts` (owns buffers, supports eliminated, free numbering, warm start, `iterateBudget`, `active` chunk mask), `loads.ts` (masses, gravity), `connectivity.ts` (tier 0 early-exit BFS -> detached components), `StressTier.ts` (tier 1 step: forces -> ratios -> damage -> tier 0; `removeBonds` for scripted cuts). Worker comes later in M3.
- `physics/` `PhysicsBackend.ts` (interface, group bits, packGroups), `RapierBackend.ts` (only Rapier importer), `fromModel.ts` (Float64 -> Float32 hull conversion).
- `integrity/islandMass.ts` mass/COM/principal inertia (Jacobi) from rest cell volumes.
- `integrity/impactLoads.ts` peak contact force per chunk between tier ticks, x2 amplification.
- `integrity/IntegrityCore.ts` message handler (TickRequest -> TickReply: broken, detached, ratio, version, returned f); `IntegrityClient.ts` (`InlineIntegrity`, `WorkerIntegrity`); `integrity.worker.ts` entry.
- `sim/` glue between integrity and physics: `IslandHandoff.ts` (static body + per-chunk colliders, detach components -> dynamic islands, grace -> debris groups), `DestructionSim.ts` (per-physics-step `tick`: drain impacts, poll reply -> breaks/handoff, send next request when none in flight; f double-buffered; `brokenLog`, `ratio`, `isStatic` for renderers).
- `render/stressColors.ts` debug heatmap: chunk colour = max bond ratio/damage.
- `render/islandPose.ts` CPU rigid placement of island chunk verts (debug until M4).
- `main.ts` (M3 app): physics.step -> sim.tick per fixed step; `syncRender` reveals broken faces, places islands, recolours bond lines by filtered ratio.
- `render/` `cullBuffers.ts` (headless outer/inner index buffers, append-only reveal), `chunkMesh.ts` (two meshes sharing one vertex buffer), `bondLines.ts` (debug lines by ratio).
- `scenes.ts` 11 demo scenes (box lists, mixed materials, `carve` for openings) + per-scene camera; `main.ts` debug app (GUI: scene, visibility, break random bonds, stats; click/space = ball aimed at cursor with gravity lead, shift+click = cut all bonds of the static chunk under the cursor). GUI folders: debug view (chunk/bond colour by ratio|damage, hide bonds below), sim (sync = `InlineIntegrity` applied same step, pause/step, time scale, physics Hz, substeps, solver ms, PCG tol, impact hold; gravity/impact threshold/Rapier iters reload), material multipliers (`scaleMaterials`, global + per-material strength, reload), stats from `TickReply.stats` (PCG iters, residual, request ms, max ratio, damaged bonds, latency). `window.demo` exposes `cur` (model/sim/physics), `load`, `throwBall` for console/automated checks.
- `tests/` model, culling, Rapier facts, solver (`solver.test.ts`, incl. `[bench]` line: `npx vitest run tests/solver.test.ts -t benchmark --reporter=verbose`).

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
- `GenericImpulseJoint` has **no per-axis motor API** in JS (only Unit/Spherical joints do) -> PLAN §4b fallback: explicit spring forces/torques per step.
- **Handles are non-integer doubles** (index+generation encoded): never store them in Int32/Uint32 arrays (silently removes the wrong collider). Use Float64Array / plain numbers.
- `setAdditionalMassProperties` takes effect only after a `step`; the principal-frame quaternion (`IslandDesc.inertiaFrame`) is honoured (tested).
- Contact-force event `dir` points from collider1 (A) to collider2 (B): force on B = +dir F, on A = -dir F. Which collider is "1" depends on creation order, so always use the sign rule.
- Mass: colliders at density 0 + `RigidBodyDesc.setAdditionalMassProperties` gives exact island mass.

## Solver (M2) facts
- Kernels take a `BondViews` bundle (views object) instead of long arg lists; scratch is module-level (allocated once). Test counts typed-array constructions = 0 per update/solve.
- Bond forces `[N,V1,V2,Tq,M1,M2]` local, N > 0 = tension. Cantilever chain matches discrete EB + shear to 1e-6 and pure EB to <1% (N=20).
- Benchmark (10x10x10 concrete grid, 2700 bonds, every 5th bond d=0.3, Node, 2026-11-14): cold 1e-8: 176 it / 57 ms; cold 1e-4: 75 it / 18 ms (~0.25 ms/it); `updateStiffness` 2.5 ms.
- Undamaged regular grids converge unrealistically fast (61 vs ~190 it on 10x10x2); benchmark with irregular damage.
- Warm start saves ~log(initial error ratio): big win at tier-1 tolerance 1e-4 for small damage steps (5 vs 157 it), ~none at 1e-8, and breaking the most-loaded bond can be *worse* than cold (104 vs 75 it @1e-4). M3 option: cold-start (or blend) after large breaks; use tol ~1e-4 for tier 1. No Cholesky fallback needed yet.

## Tier 0/1 (M3) facts
- Detached chunks stay in K (no rebuild) but get `solver.active = 0`: zero load and u, so the singular unsupported block stays at 0. Their bonds then have ratio 0.
- `StressTier.step` reassembles K whenever any live bond has filtered ratio > 1 (damage changed) — O(m) per step while overloaded; fine for now, measure in M7.
- Shear ratio uses `mu * max(0, -N/A)` (mean compressive stress) for friction; eps_p is an array input, always 0 until M4.
- `findDetached` reports any component without a support, so only pass bonds of the static structure.

## Gotchas
- Rapier `EventQueue(true)` auto-drains on every `world.step`: with substeps, call `sim.tick` after each substep or impacts are lost.
- Bond lines use additive blending: black = hidden (broken / below threshold).
- Impacts last one physics step but tier 1 low-passes ratios (~filterTime): `DestructionSim` holds each sent peak (`heldImpacts`) until a converged reply AND age >= `impactHold` (0.05 s). Without this, ball hits never broke anything.
- `IntegrityCore` must not restart PCG every request: restarted CG with a 2 ms budget stalls on cracked lattices (never converged after the first breaks). It continues while loads are bit-identical and no removals; held impacts keep f identical across requests.
- Browser check without a user: playwright-core + system Chrome (`--enable-unsafe-webgpu`, headed, off-screen window) drives `npm run dev`; keep the harness outside the repo.
- tsconfig lib has no `WebWorker` (it breaks DOM event typing). Don't use `/// <reference lib="webworker" />` either (applies program-wide); type the worker scope minimally as in `integrity.worker.ts`.
- `ChunkModel` and `MaterialTable` (Map, Infinity) structured-clone fine; worker init clones them once.
- WebGPU rejects zero-size buffers: inner index buffer capacity kept >= 3.
- Bundle is ~5 MB (three/webgpu + Rapier base64 wasm); fine for dev, code-split later (M7).
