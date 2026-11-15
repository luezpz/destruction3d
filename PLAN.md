# 10-destruction — plan v2 (Rapier does rigid bodies; we write the destruction layer)

Standalone: no imports from other projects (can be lifted out). Rapier (`@dimforge/rapier3d-compat`, Havok later)
owns rigid bodies, compounds, contacts, sleeping. Hide it behind a thin `PhysicsBackend` interface
(`createStaticChunk`, `removeCollider`, `createDynamicIsland`, `setChunkShape`, `createJoint`/`removeJoint`/`setJointTargets`, `readBodyPoses`,
`setCollisionGroups`, `drainImpactEvents`) so Havok is a swap.
We implement: **integrity simulator** (+ data-driven materials), **island splitting**, **many-bone skinning**, **fast hull refit**, **culled rendering**; Voronoi is last.
Out of scope for now: audio, dust/debris particles, save/load, networking, terrain destruction.

## Setup (M1)

Vite + TypeScript + three.js (WebGPU/TSL) + lil-gui + Rapier, own `index.html`. Layout: `model/` (ChunkModel, Material table, generators), `kernels/` (pure typed-array math),
`integrity/` (structure graph, connectivity, worker entry; external structure solver plugs in here), `physics/` (PhysicsBackend + Rapier impl), `render/` (skin shader, culling buffers), `tests/` (headless, no three.js/Rapier where possible).
Scripts: `dev`, `typecheck`, `test`, `build`. Keep a terse `NOTES.md` (architecture map, commands, decisions, gotchas) updated with each change. Units: SI, except E in GPa and strengths/stresses in MPa (converted to Pa inside kernels).

## Data flow

```
main thread                                   Worker (integrity)
Rapier: static chunk colliders + debris  --impact loads, removed bonds-->  [external structure solver] -> bonds to break
        ^                                                                    StructureGraph + Connectivity (components)
        | create/remove colliders, refit shapes                              |
        +---- islands, broken bonds, chunk poses (static graph + offsets) <--+
renderer: pose DataTexture + many-bone skinning shader
          pose sources merged on the main thread: static/"offsets" chunks from the worker, chunks on dynamic bodies
          (debris islands, promoted joint chains) from Rapier (body pose o local)
```
Worker messages use double-buffered transferable typed arrays (no SharedArrayBuffer, so no COOP/COEP headers needed).
A 1-3 frame latency is fine for fake destruction.

## 1. Data model (`ChunkModel`, headless, no three.js/Rapier)

One format, produced by every generator (grid/bricks first, Voronoi and glTF later):

```
chunks: pos(3), rest hull points + face indices, volume, material id (index into the Material table, section 2b), support flag, "skinned" flag (= material.deform === 'joints'),
        face tags (outer | inner+bondId), surface flag (has any outer face)                                 // packed arrays
bonds : a, b, area, centroid(3), normal(3), tangents, I1, I2, J, face-polygon corners, material id (interface material or derived), reinforcement
```
Float64 in the solver, Float32 only at the Rapier/GPU boundary.

## 2. Integrity simulator (background)

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

## 2b. Materials (data-driven; many types)

The strength/damage fields below are data for the (external) structure solver; with the in-house solver removed, only density (masses), `deform`, `hull` and `render` are read today.

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

Failure ratio of a bond = max over the criteria its material enables:
- tension `sigma_max/sigmaT`, compression `-sigma_min/sigmaC`, shear `tau/(tauC + mu sigma_compr)` (always);
- plastic (`sigmaY > 0`): `sigma_v/(sigmaY + H eps_p)` with von Mises `sigma_v = sqrt(sigma^2 + 3 tau^2)` at the worst corner; above 1 the rest target moves (plasticity), break at `eps_p > epsPlasticMax`;
- elastic (`sigmaY == 0`): `eps / epsElasticMax`, with `eps = max_corner |sigma| / E` (axial + bending fibre strain).
Invalid combinations are rejected when the table loads: `hull: 'refit'` needs `deform: 'joints'` (only bodies with skinned chunks change shape).
Where plasticity actually *happens* depends on `deform`: `joints` -> the promoted joint chain updates its rest targets (4b); in the static graph a plastic bond with ratio > ~0.7 triggers
promotion (4b). `offsets`/`none` with `sigmaY > 0` (e.g. clay or soft stone as dents) have no rest-target update: ratio > 1 grows damage, and `epsPlasticMax` is read as a damage limit.
Then damage as above. `propagation`: when a bond breaks, `d_neighbour += propagation` for its adjacent bonds, which makes glass shatter in a chain; 0 keeps failure local.
Grain: strength of a bond is multiplied by `lerp(1, strengthRatio, 1 - |n . axis|)`, so wood splits along the grain.
Bond materials: a bond joining chunks of different materials uses an explicit interface material if the generator provides one (mortar between bricks, steel plate on concrete);
otherwise stiffness in series `1/E = 1/E_a + 1/E_b` and the *weaker* strengths.
Which deformation path a part uses comes from `deform` (and plastic vs elastic from `sigmaY`), so a new material needs only a new record, plus a test row in the material matrix test (every preset: a cantilever under load, a compressed column, an impacted wall; no NaN, expected failure mode). The material matrix test was removed with the solver; it returns with the external solver.

## 3. Island splitting -> Rapier

This is **connected-component labeling** (union-find / BFS over intact bonds), not graph coloring (which assigns
different colors to neighbours). Run after each batch of removed bonds, local to the affected components only.

1. Label components. A component that contains a support stays the static structure (one fixed Rapier body holding all its chunk colliders, so joints can anchor to it).
   Settled debris never becomes a support (it stays a sleeping dynamic body).
   Reinforcement: a bond whose base material failed but whose `reinforcement` survives counts as intact for tier 0 *only as a hanger*: if a component is connected to a support
   solely through such bonds, it still becomes a dynamic body, attached by a ball joint (break force from the reinforcement material), contacts disabled on that joint. That is the dangling slab.
2. Every other component becomes one dynamic body: `RigidBody.dynamic` at the component COM, one convex collider per chunk
   (local translation = chunk pos - COM). Mass: set explicitly from the rest cell volumes x density (colliders at density 0 + `setAdditionalMassProperties`); hulls overlap, so Rapier's per-hull volume would double count. Inertia from the same cell volumes (sum of cell inertia + parallel axis about the island COM).
3. Velocity: 0 when it comes from the static structure; when splitting a moving debris island, `v + w x (com_child - com_parent)`.
4. Remove the detached chunks' old colliders from the static structure (`removeCollider`). Assume Rapier JS cannot re-parent a collider (verify in M1; if it can, use that),
   so cache each chunk's hull points + indices and recreate (see section 5: with indices supplied there is no quickhull cost).
5. Each new island keeps its sub-graph in the worker, so it can fracture again.
Test: mass and momentum conserved across a split; labeling of a known removal pattern; a one-support-removed wall detaches exactly its dependents.

## 4. Many-bone skinning for touching cells (mainly `deform: 'joints'`) and fake deformation (`deform: 'offsets'`)

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
  Each chunk keeps a permanent visual offset `o_i: (3,)` and tilt `th_i: (3,)` on top of its rest pose; render pose is
  `x_i = x_i0 + amplify * u_el,i + o_i`, `q_i = exp(th_i + amplify * th_el,i) q_i0` (both world-frame rotation vectors, small), with `u_el` the structure solver's displacement (sagging slabs; 0 until the external solver provides it) and `o, th` from impacts
  (the total offset is clamped to `o_max` so the render pose never strays far from the collider):
  - Impact at point `p`, unit direction `n`, impulse `J`: for chunks within graph radius `R`, `do_i = s_sat(J) phi(g_i/R) n`, `dth_i = k_t phi(g_i/R) (n x (c_i - p)) / R`,
    where `g_i` = bond-graph distance from the hit chunk, `phi(s) = (1 - s^2)^3`, `s_sat = o_max (1 - exp(-J/J_ref))` (saturates, so repeated hits dent less), `o_max` ~5-10% of chunk size.
  - Cracks fall out of the offset gradient: per bond, opening `w_b = (o_j - o_i) . n_b` (shear slip from the tangential part); `d_b = max(d_b, w_b / w_max)`.
    The dent centre moves as a block and the ring of bonds around it takes the offset difference, so ring cracks and revealed inner faces (section 6) appear around impacts; a bond that
    reaches `d >= 1` really breaks (section 2). Offsets are permanent.
  - Colliders stay at the rest pose (offsets <= `o_max`); a chunk that detaches takes its current visual pose as its body pose. Runs in the worker with the damage state, which already returns chunk poses.
  - `deform: 'joints'` materials (metals, rubber, soft plastics) get real deformation (section 4b). Real deformation of brittle materials isn't worth it: strains before cracking are millimetres.
  - `deform: 'none'` (glass): no dent; with `propagation > 0` it shatters instead.
- Candy-wrapper from linear blending only on strong twist; switch to dual quaternions if it shows.
- Tests: CSR prefix sums consistent, in-place shrink never overruns a slot, GPU and CPU skin agree for counts 1, 8, 32, partition of unity (sum w = 1), rest pose is identity, seam copies identical while intact and disjoint after a break, bent-sheet normals stay unit.

## 4b. Real bending with engine joints (`deform: 'joints'`; replaces the plastic-hinge/FK model)

Each chunk of a *bendable part* (steel/aluminium panel, post, rail, door; rubber and clay too, with `sigmaY = 0` meaning springs back and a low `sigmaY` meaning clay-like) is a dynamic Rapier body; neighbours are linked by a **compliant joint** instead of a rigid fixed joint.
- Why not a pure fixed joint: its error is the solver's residual (depends on mass, dt, iterations, joint frequency), not on a material stiffness, so
  error -> stress is not controllable and differs per chunk mass. Use a generic joint with all 6 axes motor-sprung to the rest pose with explicit
  stiffness `k` (per axis from `EA/L, G A/L, GJ/L, EI/L`, as in section 2) and damping. Then `F = k e` is a physical-ish bond force.
  **Spike result (M4, NOTES "M4 joint spike")**: Rapier JS has no 6-axis motors; explicit springs are far too soft under the cap below. Use a **spherical joint
  (translation locked) with implicit angular motors** (torsion GJ/L, bending EI/L, scaled ~1e-2 so motor compliance dominates the solver error; 4 substeps, 16 iterations);
  moments `M = k phi` from body poses; axial/shear are rigid (not measured). Plastic set = motor target angles.
- **Stiffness cap (required)**: real `EA/L` for steel is ~1e9 N/m on kg-scale chunks; no iterative/explicit spring is stable at that frequency. Cap per axis
  `k <= m_eff (omega_max)^2` with `omega_max ~ 1/(2-4 dt)`, `m_eff` the reduced mass of the pair (inertia for rotational axes). Stress is computed from the joint **force**
  `F = k e` (the load the bond actually carries in quasi-static equilibrium), not from strain times real E, so the cap only exaggerates elastic sag; yield and fracture loads stay correct.
- Error `e: (6,)` from body poses each step: `dx = R_a^T (x_b - x_a) - d0` (translation in parent frame vs rest offset `d0`), `phi = log(q0^-1 q_a^-1 q_b)` (rotation vector vs rest `q0`).
- Plasticity = rest-shape update on the joint target (VIPER-style): stress from `F = k e` mapped to `[N, V1, V2, Tq, M1, M2]` and the same corner formulas as section 2 (von Mises at the worst corner); if `sigma > sigma_y`,
  `d0 += (1 - sigma_y/sigma) dx`, `q0 <- q0 exp((1 - sigma_y/sigma) phi)`, then push the new targets to the motors. Each step, not per frame; hardening via `sigma_y + H eps_p`.
- Fracture: plastic strain above a limit, or filtered `k|e|`-based ratio > 1 -> remove the joint. The two sides are already separate bodies, so
  **no island labeling is needed for these parts**.
- Poses come for free (body transforms), colliders follow the bodies, and large bends are exact. Tier A (section 5) is automatic here; tier B adds shape change.
- Weaknesses: (1) stiff chains of iterative joints sag/stretch (more solver iterations, short chains, mass ratios <~10); (2) the part is awake while deforming, so sleep it
  once `|e|` change is tiny; (3) loads from the *static* structure (a slab resting on a steel beam) are not seen by the dynamic chain.
  Therefore scope: parts that mostly carry their own weight and impacts. For a structural beam, the integrity solver provides the interface
  bond forces as external forces on the chain's end bodies, or the beam stays on the static solve with damage only (no large bend).
- Promotion: a `joints` element lives in the static graph until impacted or its ratio exceeds ~0.7, then its chunks are promoted to dynamic bodies + joints
  (fixed body anchors for supported ends). Demotion when at rest for a while (current poses become the new rest state, plastic offsets kept).
- **Who runs it**: the joint-error/plasticity/fracture loop needs the body poses, so it runs where Rapier runs (main thread, or the physics worker if split out).
  Handshake with the integrity worker: on promotion the worker removes those chunks from the static solve and treats the interface chunks as supports (the chain carries its own load);
  on a joint break or demotion the main side sends `bondBroken`/`demote` messages so the worker's graph, weights and culling stay in sync.
- **Self-collision**: each chunk of a chain is its own body with an overlapping hull (section 5). `setContactsEnabled(false)` only covers directly jointed pairs, and collision groups are 16 bits,
  so use `partId mod P` as the group (parts sharing an id lose mutual collision, rare when far apart), or a `filterContactPair` hook comparing part ids if that proves wrong.
  Group bit budget (16 membership + 16 filter bits) is shared: e.g. bit 0 structure, bit 1 debris, bit 2 grace (Rapier notes), the remaining bits for `P` part ids.

## 5. Fast convex hull refit (materials with `hull: 'refit'` only; the rest never refit)

Never run a hull algorithm per update. **Fixed-topology hull**:
1. **Hull input = the mesh vertices the cell influences**, not a separate proxy set. Per chunk keep a vertex list `V_c = { v : w_c(v) > 0 }`
   built from the skin weights of section 4 (any influence counts). The hull is bigger than the cell and overlaps its neighbours' hulls; that is accepted.
   Overlap handling (to be tested in M1, see Rapier notes below): colliders of one body never collide with each other, and fixed-fixed pairs
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

## 6. Render culling: inner cells and inner materials

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

## 7. Execution model: stable FPS first (async, threads, WASM)

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
- **AssemblyScript (AS): the only one worth planning for.** TypeScript-like syntax compiled to WASM, Float64 and SIMD (`v128`) available, typed-array style memory, small output. Targets: sparse block mat-vec and 6x6 block ops in PCG,
  chunk-pose/skin CPU kernels, hull refit (directional extremes + convexity check), Voronoi clipping. Caveats: it is a TS *subset* (no closures over locals, no structural typing, limited generics), SIMD f64 is only 2 lanes
  (so the f64 solver gains are modest; do not drop to f32, the Float64 lesson applies to stiff solves), and threading with shared memory is weakly supported (use one WASM instance per worker instead).
  Honest expectation: 1.2-3x over well-written V8 typed-array loops, more only for SIMD-friendly kernels. **Measure the TS version first** (repo rule); port only kernels that profile hot.
- **Porffor: no for now.** Ahead-of-time JS/TS to WASM, but experimental and incomplete; fine to watch, not to depend on for a game's core loop.
- **Javy: no.** It embeds a JS engine (QuickJS) inside WASM for sandboxing/portability, so it runs *slower* than V8; it solves a different problem.
- Fallback if AS limits bite: Rust (or C) to WASM for the same kernels; the boundary is identical.
- **How we keep the option cheap**: kernels live in `kernels/` with a TS reference and, when needed, an AS build behind the same signature `(memory views, offsets, counts) -> void`.
  One differential test runs both implementations on random inputs (tolerance 1e-9 in f64) and a micro-benchmark records the speed ratio in the notes, so a port only lands with a measured gain.

## Rapier overlap notes (test in M1 before building on them)
- Same-body colliders: no contacts between them (compound is safe). Fixed-vs-fixed: no contacts by default.
- Jointed bodies: `joint.setContactsEnabled(false)` for the rebar-dangling case.
- Gap: a freshly detached island overlaps the remaining structure and its sibling islands at the crack. Active contacts would push out violently
  (AGENTS lesson: cap depenetration). Plan: put new islands in a temporary collision group that ignores their former neighbours,
  and restore the normal group once separated (cheap AABB/distance check per island, or after a fixed time). Collision groups are per collider,
  not per pair; if a pair rule is needed use physics hooks (`filterContactPair`), which cost a JS call per pair, so keep that as the fallback.
- Chunks of a debris island that later re-fractures overlap too; the same grace group applies.

## Milestones

**M1 Skeleton.** `ChunkModel`, a box/brick-grid generator (bonds from face overlap), Rapier harness with the backend interface,
the `Material` table with presets (section 2b), project setup (above), debug render (chunk colors, bond lines coloured by ratio), face tags + surface flag, and the culled two-material render (section 6) from the start, since it shapes the index buffers. *No Voronoi needed to proceed.*
**M2 Bond solver** — *removed 2026-11-14 (replaced by an external solver, section 2)*. Was: (headless, written as pure typed-array kernels in `kernels/`, section 7, with a micro-benchmark for the AS decision): axial bar vs `FL/(EA)`; cantilever vs Euler-Bernoulli; free island equilibrium; warm-start speed-up; local effect of a break; no per-solve allocation. Measure PCG iterations at ~1000 chunks.
**M3 Core loop.** Worker + tier 0, component labeling, island handoff to Rapier. Done when removing a wall drops the dependents instantly. (Tier 1 stress/damage, failure ratios, impact loads and the material matrix test were removed with the solver; they come back with the external solver.)
**M3b External structure solver integration** (when available): wire it into `IntegrityCore` per the section 2 contract; restore impact loads, overload-driven breaks, the material matrix test, and the stress/damage debug views.
**M4 Skinning + deformation** (sections 4, 4b): fake offsets for `offsets` materials (impact dents, ring cracks); `joints` materials (steel panels/posts) as joint chains with the promotion handshake, plasticity by rest-target update, skinned from body poses. Tests: cantilever sag vs Euler-Bernoulli (expect some error; record it), permanent set after unload, fracture removes the joint and separates bodies, stability over 10 s.
**M5 Hull refit** (section 5) with the tier A/B toggle and budget GUI.
**M6 Voronoi (non-priority):** per site clip the element's convex volume by bisector planes of its k nearest sites; the clipped polygon on each plane is the bond face, so adjacency/area/moments come free. Graded density, Poisson-disk sites. Concave buildings are built from convex elements (columns, slabs, panels around openings), each fractured in its own volume, bonded by coplanar face overlap. Blender route: any splitter -> glTF, one node per chunk + `extras`; importer builds bonds by AABB overlap + coplanar opposite-normal face clipping. Tests: volumes sum to the source (1e-9), symmetric adjacency.
**M7 Game scene + perf** (frame-budget caps and queues from section 7, worker/SAB upgrades and AS ports only where profiling justifies them): level, wrecking ball/explosive, GUI, debris cap + despawn/merge of tiny far pieces, profiling before optimizing.

## Risks
1. External structure solver integration: data contract (section 2), cost per tick under the frame budget, and how its outputs feed 4/4b (promotion, offsets).
2. Joint-chain plasticity (4b) is the least proven part: stiffness/stability of iterative joints, the rest-target update, and the promotion/demotion handshake with the integrity worker. Validate on a cantilever first.
   Related: three deformation sources (static solve, fake offsets, joint chains) must agree on the render pose at promotion/demotion; test for pops.
3. (If the external solver is a small-displacement linear solve) no P-delta: slender columns won't buckle by themselves; add an Euler-load check per column if the game needs it.
4. Rapier JS API details (convexMesh, setShape, contact-force events, re-parenting) must be checked against the installed version in M1.
5. Overlapping hulls (`w > 0` influence): cross-body overlap after splits (grace group, Rapier notes) and chain self-collision (4b) are the practical hazards; solver is Float64 but Rapier/GPU are Float32, so keep poses relative to the island/structure origin to limit precision loss on large levels.
6. Collider recreation on detach (hundreds of colliders in one frame): budget/spread across frames if it spikes.
