# PROGRESS

Status tracker across sessions. Update at the end of every session. Spec: `PLAN.md`. Rules: `AGENTS.md`.

**Current milestone:** M3 code-complete; app (now using the worker) never viewed in a browser (needs user)
**Next task:** User: run `npm run dev`, throw balls / break bonds, report issues (worker only verified by build + inline/structured-clone tests). Then M4 spike (PLAN §4b, three options listed below).
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

## M2 Bond solver (headless kernels) — PLAN §2, §7
- [x] Bond stiffness `K_b`, CSR 6x6 assembly
- [x] Block-Jacobi PCG, warm start, time-sliced
- [x] Tests: axial bar, cantilever vs Euler-Bernoulli, free-island equilibrium, warm-start speed-up, local break effect, no per-solve allocation
- [x] Benchmark: PCG iterations @ ~1000 chunks (record in NOTES.md)

## M3 Core loop (brittle collapse)
- [x] Integrity worker + messaging (double buffers, versions) — not yet run in a browser
- [x] Tier 0 connectivity, tier 1 stress/damage, failure ratios (reinforcement hangers + plastic eps_p update deferred)
- [x] Material matrix test
- [x] Component labeling + island handoff to Rapier (mass/inertia, grace group) — headless + Rapier test; not in the app yet
- [x] Impact loads from contact events (peak per tier tick, x2 amplification; `dir` sign verified)
- [x] Done-when (headless tests): wall removal drops dependents; overloaded cantilever cracks + falls after delay. Visual check pending.

## M4 Skinning + deformation — PLAN §4, §4b
- [ ] Variable-length weights (CSR), pose texture, TSL skin + CPU reference
- [ ] Fake offsets (dents, ring cracks)
- [ ] Spike first (cantilever chain, 10 s): (a) explicit spring forces on Rapier, (b) fixed joints + calibrated residual error as deformation (user idea, hacky), (c) Jolt `SixDOFConstraint` spring motors if a/b fail. Pick by stability + sag error; record in NOTES.
- [ ] Joint chains, promotion/demotion handshake, plasticity
- [ ] Tests per PLAN §4 / M4

## M5 Hull refit — PLAN §5
## M6 Voronoi — PLAN M6
## M7 Game scene + perf — PLAN §7, M7

## Decisions / deviations log
<!-- YYYY-MM-DD: decision — reason -->
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
- 2026-11-14: Read PLAN.md, set up agent docs.
- 2026-11-14: M1 implemented: model, materials, generators, culling, Rapier backend + facts tests (20 passing), debug app. Build OK. Not yet viewed in a browser.
- 2026-11-14: M3 worker: IntegrityCore (message handler), Inline/WorkerIntegrity clients, integrity.worker.ts, DestructionSim one-request-in-flight; 83 tests pass, build emits worker chunk.
- 2026-11-14: M3 impacts (ImpactLoads), DestructionSim main-thread loop, CPU island placement (render/islandPose), main.ts wired; 80 tests pass, build OK.
- 2026-11-14: M3 material matrix (36 cases), islandMass (principal inertia), IslandHandoff (sim/), IslandDesc.inertiaFrame; 74 tests pass, build OK.
- 2026-11-14: M3 tier 0/1: failureRatio + damage kernels, Connectivity, StressTier, BondSolver.active mask; 7 tests (34 total) pass.
- 2026-11-14: M2 implemented: bond stiffness, block CSR, PCG, inertial relief, BondSolver; 7 solver tests (27 total) pass; benchmark in NOTES.
