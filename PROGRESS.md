# PROGRESS

Status tracker across sessions. Update at the end of every session. Spec: `PLAN.md`. Rules: `AGENTS.md`.

**Current milestone:** M4 — joint spike done (spherical joints + implicit angular motors chosen). In-house structure solver removed 2026-11-14 (external solver comes later, M3b).
**Next task:** remaining M4: dent tuning (cascades from debris). Open: bake ratchet of loop parts under sustained load above yield (NOTES). Promotion by stress ratio waits for the external solver (M3b). Consider evaluating Rapier 0.21 FEM soft bodies (NOTES) before investing more in joint chains.
**Blockers:** none

## M1 Skeleton
- [x] Project setup + folder layout + `NOTES.md`
- [x] `ChunkModel` packed arrays (chunks, bonds, face tags, surface flag) — PLAN §1
- [x] Material table + presets + load-time validation — PLAN §2b
- [x] Box/brick-grid generator, bonds from face overlap
- [x] `PhysicsBackend` interface + Rapier impl (joints: fixed/ball only; spring targets deferred to M4)
- [x] Verify Rapier API (results in NOTES.md). Still open: does `convexMesh` skip quickhull internally, `setShape` cost (→ M5)
- [x] Debug render (untested in browser): chunk colours, bond lines coloured by ratio
- [x] Culled two-material render (untested in browser) (outer static IB, inner append-only IB, `drawRange`) — PLAN §6
- [x] Tests: generator adjacency, culling (initial = outer only, append idempotent, monotonic, brute-force match)

## M2 Bond solver (headless kernels) — PLAN §2, §7 — REMOVED 2026-11-14 (code in git history before this date)
- [x] Bond stiffness `K_b`, CSR 6x6 assembly
- [x] Block-Jacobi PCG, warm start, time-sliced
- [x] Tests: axial bar, cantilever vs Euler-Bernoulli, free-island equilibrium, warm-start speed-up, local break effect, no per-solve allocation
- [x] Benchmark: PCG iterations @ ~1000 chunks (record in NOTES.md)

## M3 Core loop (brittle collapse)
- [x] Integrity worker + messaging (double buffers, versions) — not yet run in a browser
- [x] Tier 0 connectivity (now `StructureGraph`). Tier 1 stress/damage + failure ratios were done, removed 2026-11-14
- [ ] ~~Material matrix test~~ removed with the solver; returns in M3b
- [x] Component labeling + island handoff to Rapier (mass/inertia, grace group) — headless + Rapier test; not in the app yet
- [ ] ~~Impact loads from contact events~~ removed with the solver (backend `drainImpactEvents` + `dir` sign rule kept); returns in M3b
- [x] Done-when: wall removal drops dependents (headless test). Overload-driven cracking removed with the solver.

## M3b External structure solver integration — PLAN §2 contract
- [ ] Plug the external solver into `IntegrityCore` (reads `StructureGraph` + materials + masses + loads, returns bonds to break)
- [ ] Restore impact loads, material matrix test, stress/damage debug views (chunk/bond heatmaps, solver stats)

## M4 Skinning + deformation — PLAN §4, §4b
- [x] Variable-length weights (CSR, topology-aware, in-place reweight on break) + CPU skin reference (`model/skinWeights.ts`, `kernels/skin.ts`, `tests/skin.test.ts`)
- [x] Pose texture + TSL skin (`render/skinMaterial.ts`), wired into `chunkMesh`/`main.ts`; islands posed via `writeIslandPoses` (CPU vertex placement removed). Browser-verified (concrete wall, new 'steel plate' scene: cut + knock-off, clean cut edge)
- [x] Automated GPU vs CPU skin agreement for counts 1, 8, 32: `npm run test:gpu` (WebGPU + WebGL2 fallback, max err ~1.3e-6)
- [x] Fake offsets (dents, ring cracks): `integrity/DentField.ts` in the worker, `Material.dent` record, impacts in TickRequest, poses in TickReply -> static chunk poses (`tests/dent.test.ts`). Browser-verified on 'concrete wall'. Elastic `u_el` term waits for M3b
- [x] Spike (cantilever chain, 10 s): (a) explicit springs, (b) fixed joints, (c) motors — found in Rapier itself (spherical joint angular motors), no Jolt needed. Chosen (c); numbers in NOTES "M4 joint spike". Backend: `createSpringJoint`, `setSpringTarget`, `readBodyStates`, `applyWrenches`.
- [x] Joint chains (`sim/JointChains.ts`, `kernels/jointPlasticity.ts`): promotion on impact (>= 2000 N) or script, one body per chunk, spring joints (anchored to the static body at the part's edge), plasticity by motor-target update, fracture -> bond removal to integrity; `IslandHandoff.detach` skips chunks already on bodies. Browser-verified on 'steel plate' (strength x0.01: smooth plastic bend)
- [x] Joint-chain perf: body damping + own rest test -> sleep (hit plate asleep in ~3 s, 0.4 ms/frame after), part collision groups, rest-state bake fixing endless jitter of plastically dented plates (`tests/chainPerf.test.ts`, NOTES "Joint-chain perf")
- [x] Demotion (at rest -> static colliders at the deformed pose, plastic set kept, re-promotion resumes without pop; no worker message needed until M3b). Browser-verified on 'steel plate' (hit -> 90 bodies demoted after settling, re-hit re-promotes, bends further, demotes again)
- [x] Tests (`tests/jointChains.test.ts`): kernel yield return, steel stable 10 s (sag 8.2 mm), permanent set after unload (34.5 of 35.1 cm kept; elastic springs back to -1.4 mm), fracture removes joint + bond, chain falls, sag vs hinge-chain/Euler-Bernoulli (sim 5.9 mm = 1.36x discrete hinge at k x1e-2; raw EB 0.038 mm; NOTES)

## M5 Hull refit — PLAN §5
## M6 Voronoi — PLAN M6
## M7 Game scene + perf — PLAN §7, M7

## Decisions / deviations log
<!-- YYYY-MM-DD: decision — reason -->
- 2026-11-15: Fake dents: `Material.dent` record (oMax/jRef/radius/tilt/wMax/minForce); saturation on accumulated impulse per hit chunk (contact split into substep events); crack measure |o_a - o_b| (opening + slip); contact point = chunk centre; tilt scaled by s/(R h)^2 (PLAN gives no impulse scaling).
- 2026-11-15: Demotion: islands get per-chunk local poses + chunks a static pose (`setPos/setQ`), `IslandHandoff.attach`; no `demote` message (graph never saw promotion). Rest-state bake restricted to parts with loops (tree parts ratcheted under gravity). Re-promotion restores stored targets instead of baking.
- 2026-11-15: Joint chains: absolute body damping + own sleep (Rapier cost is the iterations; settling is the lever), part bit as sole membership, rest-state bake after each yield episode (loop incompatibility in 2D parts). Costs: steel sag 5.9 vs 8.2 mm, residual elastic stress baked into permanent set. Kept 16 iters x 4 substeps.
- 2026-11-15: Island re-fracture supported (supersedes "island re-fracture not supported" below): integrity processes bonds of floating parts too; TickReply gains `floatChunks/floatOffset`.
- 2026-11-15: Joint-chain motor targets are additive per-axis rotation vectors (bond frame), matching Rapier's per-axis motor angles; fine for moderate bends, revisit for crumpling. Promotion = one single-chunk island per chunk (reuses handoff, render and grace logic). App defaults now 4 substeps / 16 iterations (chains need them).
- 2026-11-15: Skin data in data textures, not storage buffers — one path for WebGPU and the WebGL2 fallback. Whole-texture uploads for now (tiny).
- 2026-11-15: PLAN consistency pass — §4b rewritten to the spike result (spherical joint + 3 angular motors, error `phi: (3,)`, rotational plasticity only, stiffness scaled ~1e-2 instead of the explicit-spring cap); corner-stress formulas moved into §2b (§2 no longer has them); backend method list matches `PhysicsBackend.ts`; M1-verified Rapier facts marked verified; not-yet-wired parts (impact loads, worker poses, island re-fracture, ratio-based promotion) flagged.
- 2026-11-14: Removed the in-house structure solver (bond stiffness, PCG, inertial relief, failure ratios, damage, StressTier/BondSolver, impact loads, stress heatmaps, solver GUI/stats, solver/integrity/materialMatrix/impact tests) — results/feel not as wanted; an external solver will take the structure graph (mass + material data) as input. Kept: ChunkModel, materials, `StructureGraph` (alive + adjacency + tier 0), worker boundary, island handoff, joint spike. Supersedes the solver/impact entries below.
- 2026-11-14: 4b joints = spherical impulse joint + implicit angular motors (stiffness x1e-2, 4 substeps, 16 iters) — explicit springs ~1e5x too soft under the stability cap; fixed joints give uncalibrated stress and no plastic target. Supersedes "explicit forces/torques" below.
- 2026-11-14: Impact peaks held >= 0.05 s and until a converged solve (was: one request) — single-step loads were filtered away; PCG continues across requests when loads are unchanged (supersedes "always restart") — restarts stalled convergence after breaks.
- 2026-11-14: Worker always restarts PCG on the latest loads each request (warm start keeps progress) so long solves never hide new impacts; scripted removals are queued and applied by the next request (one tick latency). Island re-fracture not supported yet (handoff skips components already debris).
- 2026-11-14: Moving islands rendered by CPU rigid placement into the shared vertex buffer (debug path; replaced by M4 pose texture). Tier 1 runs only on converged solves (main thread v1).
- 2026-11-14: New `src/sim/` layer glues integrity + physics (both allowed there); grace window 0.1 s (longer lets islands sink into ledges). Material-matrix wall case asserts "a highly loaded bond fails", not the probe argmax (damage redistributes load in indeterminate structures).
- 2026-11-14: Warm-start test uses tier-1 tolerance 1e-4 (no gain at 1e-8); after large breaks warm can lose to cold — decide restart policy in M3.
- 2026-11-14: 4b joint springs use explicit forces/torques (Rapier JS GenericImpulseJoint has no per-axis motors).
- 2026-11-14: Code under `src/` (PLAN folders as `src/model` etc.); vitest for tests; tsc 7.
- 2026-11-14: Added `createGround`/`createSphere` to PhysicsBackend for demo/projectiles.
- 2026-11-14: Created AGENTS.md + PROGRESS.md; no code yet.

## Session log
<!-- YYYY-MM-DD: what was done, what's next -->
- 2026-11-15: GPU skin harness (`npm run test:gpu`, playwright-core devDep, `skincheck.html`): WebGPU and WebGL2 match the CPU kernel to ~1e-6 (counts 1/8/32, positions + normals). WebGL2 fallback now verified. 58 tests, typecheck + build OK.
- 2026-11-15: M4 chain sag vs hinge/EB test recorded (sim 1.36x softer than ideal hinge chain); 58 tests pass, typecheck OK.
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
- 2026-11-14: Read PLAN.md, set up agent docs.
- 2026-11-14: M1 implemented: model, materials, generators, culling, Rapier backend + facts tests (20 passing), debug app. Build OK. Not yet viewed in a browser.
- 2026-11-14: M3 worker: IntegrityCore (message handler), Inline/WorkerIntegrity clients, integrity.worker.ts, DestructionSim one-request-in-flight; 83 tests pass, build emits worker chunk.
- 2026-11-14: M3 impacts (ImpactLoads), DestructionSim main-thread loop, CPU island placement (render/islandPose), main.ts wired; 80 tests pass, build OK.
- 2026-11-14: M3 material matrix (36 cases), islandMass (principal inertia), IslandHandoff (sim/), IslandDesc.inertiaFrame; 74 tests pass, build OK.
- 2026-11-14: M3 tier 0/1: failureRatio + damage kernels, Connectivity, StressTier, BondSolver.active mask; 7 tests (34 total) pass.
- 2026-11-14: M2 implemented: bond stiffness, block CSR, PCG, inertial relief, BondSolver; 7 solver tests (27 total) pass; benchmark in NOTES.
