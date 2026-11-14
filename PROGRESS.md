# PROGRESS

Status tracker across sessions. Update at the end of every session. Spec: `PLAN.md`. Rules: `AGENTS.md`.

**Current milestone:** M1 Skeleton (code complete; awaiting visual check in browser)
**Next task:** visually verify `npm run dev` (WebGPU render, reveal, ball impacts), then M2 bond stiffness kernel + CSR assembly.
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
- [ ] Bond stiffness `K_b`, CSR 6x6 assembly
- [ ] Block-Jacobi PCG, warm start, time-sliced
- [ ] Tests: axial bar, cantilever vs Euler-Bernoulli, free-island equilibrium, warm-start speed-up, local break effect, no per-solve allocation
- [ ] Benchmark: PCG iterations @ ~1000 chunks (record in NOTES.md)

## M3 Core loop (brittle collapse)
- [ ] Integrity worker + messaging (double buffers, versions)
- [ ] Tier 0 connectivity, tier 1 stress/damage, failure ratios
- [ ] Material matrix test
- [ ] Component labeling + island handoff to Rapier (mass/inertia, grace group)
- [ ] Impact loads from contact events
- [ ] Done-when: wall removal drops dependents; overloaded cantilever cracks + falls after delay

## M4 Skinning + deformation — PLAN §4, §4b
- [ ] Variable-length weights (CSR), pose texture, TSL skin + CPU reference
- [ ] Fake offsets (dents, ring cracks)
- [ ] Joint chains, promotion/demotion handshake, plasticity
- [ ] Tests per PLAN §4 / M4

## M5 Hull refit — PLAN §5
## M6 Voronoi — PLAN M6
## M7 Game scene + perf — PLAN §7, M7

## Decisions / deviations log
<!-- YYYY-MM-DD: decision — reason -->
- 2026-11-14: 4b joint springs use explicit forces/torques (Rapier JS GenericImpulseJoint has no per-axis motors).
- 2026-11-14: Code under `src/` (PLAN folders as `src/model` etc.); vitest for tests; tsc 7.
- 2026-11-14: Added `createGround`/`createSphere` to PhysicsBackend for demo/projectiles.
- 2026-11-14: Created AGENTS.md + PROGRESS.md; no code yet.

## Session log
<!-- YYYY-MM-DD: what was done, what's next -->
- 2026-11-14: Read PLAN.md, set up agent docs.
- 2026-11-14: M1 implemented: model, materials, generators, culling, Rapier backend + facts tests (20 passing), debug app. Build OK. Not yet viewed in a browser.
