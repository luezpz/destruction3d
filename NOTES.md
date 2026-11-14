# NOTES

## Commands
- `npm run dev` (Vite, debug app at /), `npm run typecheck` (tsc 7, noEmit), `npm test` (vitest, Node), `npm run build`.

## Architecture map (`src/`)
- `model/` headless data: `ChunkModel.ts` (packed SoA format, flags, OUTER_FACE), `Material.ts` (records, presets, validation, derived interface materials), `boxGenerator.ts` (boxes -> ChunkModel: bonds from face overlap, face splitting + tags), `generators.ts` (grid, running-bond brick wall).
- `kernels/` pure typed-array math (empty until M2).
- `integrity/` solver/damage/connectivity/worker (empty until M2/M3).
- `physics/` `PhysicsBackend.ts` (interface, group bits, packGroups), `RapierBackend.ts` (only Rapier importer), `fromModel.ts` (Float64 -> Float32 hull conversion).
- `render/` `cullBuffers.ts` (headless outer/inner index buffers, append-only reveal), `chunkMesh.ts` (two meshes sharing one vertex buffer), `bondLines.ts` (debug lines by ratio).
- `scenes.ts` demo structures, `main.ts` debug app (GUI: scene, visibility, break random bonds, throw ball (space), stats).
- `tests/` model, culling, Rapier facts.

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
- Mass: colliders at density 0 + `RigidBodyDesc.setAdditionalMassProperties` gives exact island mass.

## Gotchas
- tsconfig lib has no `WebWorker` (it breaks DOM event typing); worker files will need `/// <reference lib="webworker" />`.
- WebGPU rejects zero-size buffers: inner index buffer capacity kept >= 3.
- Bundle is ~5 MB (three/webgpu + Rapier base64 wasm); fine for dev, code-split later (M7).
