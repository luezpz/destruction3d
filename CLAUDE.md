# CLAUDE.md — destruction3d

Real-time structural destruction layer on top of Rapier (rigid bodies) + three.js (WebGPU/TSL).
`NOTES.md` is the single memory file: status/next task, architecture map, commands, decisions, gotchas, and the design spec ("Design §N", source of truth for *what* and *why*). Don't restate it; cite sections (e.g. "Design §4b").

## Session start (stop when you have enough)
1. `NOTES.md` "Status" — current milestone, next task, blockers. **Start here.**
2. `NOTES.md` architecture map, commands, gotchas.
3. Only the Design section relevant to the current task.

## Session end (required)
- Update `NOTES.md` in the same change that makes it stale: tick finished milestone items, set Status "Next", log blockers/deviations with date (YYYY-MM-DD), add a Session log line.
- If the design changes, edit the Design spec and add one line to the Decisions log.

## Hard rules
- **Standalone**: no imports from other repos/projects.
- **Rapier only behind `PhysicsBackend`** (`physics/`). Nothing outside `physics/` imports `@dimforge/rapier3d-compat` (Havok must be a swap).
- **Headless core**: `model/`, `kernels/`, `integrity/` never import three.js or Rapier. Tests for them run in Node.
- **Kernels** (`kernels/`): pure functions over typed arrays, SoA, no classes in hot loops, **no allocation per call**, signature `(views, offsets, counts) -> void`. Must run unchanged in main, worker, or WASM.
- **Precision**: Float64 in the solver; Float32 only at the Rapier/GPU boundary. Keep poses relative to the structure/island origin.
- **Units**: SI, except E/H in GPa and strengths/stresses in MPa; convert to Pa inside kernels only.
- **Data-driven materials**: never branch on material names ("concrete", "steel"). Branch on record fields (`sigmaY`, `deform`, `hull`, ...). New material = new record (+ row in the material matrix test once it returns with the external solver, NOTES M3b).
- **Frame never waits** on destruction work: main thread does bounded, capped work; heavy work is budgeted queues or workers. Results carry a graph version; breaks beat stale data.
- Worker messaging: transferable double buffers; no SharedArrayBuffer (unless the `SharedState` upgrade is explicitly started).
- Measure before optimizing; AssemblyScript ports only with a differential test + benchmark recorded in `NOTES.md`.

## Code style
- Small single-purpose modules, folders by domain: `src/{model,kernels,integrity,physics,render}/`, tests in `tests/`.
- Every function: 1-3 line doc (purpose, params with type/meaning, return). Explain why.
- Math: state the formula and shapes (e.g. `K_b: (12,12)`, `u: (6n,)`) and variable meanings, as NOTES.md does.
- Reuse existing helpers; match conventions in nearby code.

## Verify before claiming done
`npm run typecheck && npm run test` (and `npm run build` for render/setup changes). Report failures with output; never tick a NOTES milestone item if they fail.
Each milestone's tests are listed in NOTES.md "Milestones" and per-section "Tests:" lines — they are acceptance criteria.

## Known risks to keep in mind (NOTES Design "Risks")
External structure solver integration (Design §2 contract); joint-chain stability (4b); no P-delta buckling; Rapier JS API details still unverified (does `convexMesh` skip quickhull, `setShape` cost) — verify against the installed version and record results in `NOTES.md`.
