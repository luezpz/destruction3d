# AGENTS.md — destruction3d

Real-time structural destruction layer on top of Rapier (rigid bodies) + three.js (WebGPU/TSL).
`PLAN.md` is the design spec (source of truth for *what* and *why*). Don't restate it; cite sections (e.g. "PLAN §4b").

## Session start (read in this order, stop when you have enough)
1. `PROGRESS.md` — current milestone, next task, open blockers. **Start here.**
2. `NOTES.md` — architecture map, commands, decisions, gotchas (created in M1).
3. Only the `PLAN.md` section relevant to the current task.

## Session end (required)
- Update `PROGRESS.md`: tick finished items, set "Next", log blockers/deviations with date (YYYY-MM-DD).
- Update `NOTES.md` in the same change that makes it stale.
- If the design changes, edit `PLAN.md` and add one line to the Decisions log in `PROGRESS.md`.

## Hard rules
- **Standalone**: no imports from other repos/projects.
- **Rapier only behind `PhysicsBackend`** (`physics/`). Nothing outside `physics/` imports `@dimforge/rapier3d-compat` (Havok must be a swap).
- **Headless core**: `model/`, `kernels/`, `integrity/` never import three.js or Rapier. Tests for them run in Node.
- **Kernels** (`kernels/`): pure functions over typed arrays, SoA, no classes in hot loops, **no allocation per call**, signature `(views, offsets, counts) -> void`. Must run unchanged in main, worker, or WASM.
- **Precision**: Float64 in the solver; Float32 only at the Rapier/GPU boundary. Keep poses relative to the structure/island origin.
- **Units**: SI, except E/H in GPa and strengths/stresses in MPa; convert to Pa inside kernels only.
- **Data-driven materials**: never branch on material names ("concrete", "steel"). Branch on record fields (`sigmaY`, `deform`, `hull`, ...). New material = new record (+ row in the material matrix test once it returns with the external solver, PLAN M3b).
- **Frame never waits** on destruction work: main thread does bounded, capped work; heavy work is budgeted queues or workers. Results carry a graph version; breaks beat stale data.
- Worker messaging: transferable double buffers; no SharedArrayBuffer (unless the `SharedState` upgrade is explicitly started).
- Measure before optimizing; AssemblyScript ports only with a differential test + benchmark recorded in `NOTES.md`.

## Code style
- Small single-purpose modules, folders by domain: `src/{model,kernels,integrity,physics,render}/`, tests in `tests/`.
- Every function: 1-3 line doc (purpose, params with type/meaning, return). Explain why.
- Math: state the formula and shapes (e.g. `K_b: (12,12)`, `u: (6n,)`) and variable meanings, as PLAN.md does.
- Reuse existing helpers; match conventions in nearby code.

## Verify before claiming done
`npm run typecheck && npm run test` (and `npm run build` for render/setup changes). Report failures with output; never mark a PROGRESS item done if they fail.
Each milestone's tests are listed in PLAN.md "Milestones" and per-section "Tests:" lines — they are acceptance criteria.

## Known risks to keep in mind (PLAN "Risks")
External structure solver integration (PLAN §2 contract); joint-chain stability (4b); no P-delta buckling; Rapier JS API details unverified (convexMesh, setShape, contact-force events, re-parenting, 6-axis motors) — verify against the installed version and record results in `NOTES.md`.
