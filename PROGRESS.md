# PROGRESS

Status tracker across sessions. Update at the end of every session. Spec: `PLAN.md`. Rules: `AGENTS.md`.

**Current milestone:** M4 — joint spike done (spherical joints + implicit angular motors chosen). In-house structure solver removed 2026-11-14 (external solver comes later, M3b).
**Next task:** M4 skinning (weights CSR, pose texture) and joint chains (plasticity loop, joint fracture). Promotion by stress ratio waits for the external solver (M3b); until then promote on impact/scripted.
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
- [ ] Variable-length weights (CSR), pose texture, TSL skin + CPU reference
- [ ] Fake offsets (dents, ring cracks)
- [x] Spike (cantilever chain, 10 s): (a) explicit springs, (b) fixed joints, (c) motors — found in Rapier itself (spherical joint angular motors), no Jolt needed. Chosen (c); numbers in NOTES "M4 joint spike". Backend: `createSpringJoint`, `setSpringTarget`, `readBodyStates`, `applyWrenches`.
- [ ] Joint chains, promotion/demotion handshake, plasticity
- [ ] Tests per PLAN §4 / M4

## M5 Hull refit — PLAN §5
## M6 Voronoi — PLAN M6
## M7 Game scene + perf — PLAN §7, M7

## Decisions / deviations log
<!-- YYYY-MM-DD: decision — reason -->
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
