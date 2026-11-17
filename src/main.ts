// Debug app: generated structure, culled outer/inner render, bond lines, Rapier, and the M3 core loop (DestructionSim):
// scripted bond removals -> connectivity -> falling islands (no structure solver yet). Chunks are skinned on the GPU from the pose texture (render/skinMaterial.ts).

import * as THREE from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import GUI from 'lil-gui';
import { ChunkModel, surfaceChunkCount } from './model/ChunkModel';
import { MATERIAL_PRESETS, MaterialScale, UNIT_SCALE, createMaterialTable, scaleMaterials, type MaterialTable } from './model/Material';
import { buildCullBuffers } from './render/cullBuffers';
import { ChunkMeshes, createChunkMeshes, revealBondFaces } from './render/chunkMesh';
import { BondLines, createBondLines, updateBondColors } from './render/bondLines';
import { packGroups } from './physics/PhysicsBackend';
import { RapierBackend, initRapier } from './physics/RapierBackend';
import { DestructionSim } from './sim/DestructionSim';
import { InlineIntegrity, WorkerIntegrity } from './integrity/IntegrityClient';
import { writeIslandPoses } from './render/islandPose';
import { buildBondAdjacency, BondAdjacency } from './kernels/bondAdjacency';
import { buildSkinWeights, reweightBroken, SkinWeights } from './model/skinWeights';
import { writeChunkPose, writeIdentityPoses } from './kernels/skin';
import { createSkinTextures, SkinTextures, uploadDirtyWeights, writeSkinRanges } from './render/skinMaterial';
import { SCENES, SceneName, buildScene } from './scenes';

const ALL = packGroups(0xffff, 0xffff);

/** Everything that belongs to the currently loaded structure. */
interface Loaded {
  model: ChunkModel;
  table: MaterialTable;
  meshes: ChunkMeshes;
  bonds: BondLines;
  broken: Uint8Array;
  physics: RapierBackend;
  sim: DestructionSim;
  /** brokenLog entries already revealed/coloured. */
  shownBroken: number;
  balls: { body: number; mesh: THREE.Mesh }[];
  /** Skin state: render-side bond adjacency + alive copy (from brokenLog), weights, pose/entry textures. */
  skin: { adj: BondAdjacency; alive: Uint8Array; sw: SkinWeights; tex: SkinTextures };
}

async function main(): Promise<void> {
  await initRapier();
  const renderer = new THREE.WebGPURenderer({ antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  renderer.setSize(innerWidth, innerHeight);
  renderer.shadowMap.enabled = true;
  document.body.appendChild(renderer.domElement);
  await renderer.init();

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1d2026);
  const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.05, 500);
  camera.position.set(5, 3.5, 7);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 1.5, 0);
  controls.update();

  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x3a3428, 1.2));
  const sun = new THREE.DirectionalLight(0xffffff, 2.5);
  sun.position.set(6, 10, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -8, right: 8, top: 8, bottom: -8 });
  scene.add(sun);
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.MeshStandardNodeMaterial({ color: 0x3b3f45, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  const params = {
    scene: SCENES[0] as SceneName, outer: true, inner: true, bondLines: false, breakCount: 10, ballSpeed: 20, ballRadius: 0.25, impactEnergy: 1, breakMinSpeed: 0.5, dents: true,
    // Sim (live unless noted "reload")
    sync: false, paused: false, timeScale: 1, physicsHz: 60, substeps: 4, // joint chains need 4 substeps + 16 iterations (NOTES "M4 joint spike")
    g: 9.81, impactThreshold: 500, solverIterations: 16, pgsIterations: 1, // reload
  };
  const matScale: MaterialScale = { ...UNIT_SCALE };
  const perStrength: Record<string, number> = Object.fromEntries(MATERIAL_PRESETS.map((m) => [m.name, 1]));
  const stats = {
    chunks: 0, surface: 0, bonds: 0, outerTris: 0, innerTris: 0, brokenBonds: 0, islands: 0, joints: 0, brokenJoints: 0, collisionBreaks: 0,
    mode: '', version: 0, simTickMs: '', replies: 0, latencySteps: 0,
  };
  let cur: Loaded | null = null;

  /** Disposes the current structure and loads the selected scene. */
  function load(): void {
    if (cur) {
      for (const o of [cur.meshes.outer, cur.meshes.inner, cur.bonds.lines, ...cur.balls.map((b) => b.mesh)]) {
        scene.remove(o);
        o.geometry.dispose();
      }
      cur.sim.dispose();
      cur.physics.dispose();
    }
    const table = createMaterialTable(scaleMaterials(MATERIAL_PRESETS, matScale, perStrength));
    const demo = buildScene(params.scene, table);
    const model = demo.model;
    camera.position.set(...demo.camera);
    controls.target.set(...demo.target);
    controls.update();
    const adj = buildBondAdjacency(model.chunkCount, model.bondA, model.bondB, model.bondCount);
    const alive = new Uint8Array(model.bondCount).fill(1);
    const sw = buildSkinWeights(model, adj, alive);
    const tex = createSkinTextures(model.chunkCount, sw);
    writeIdentityPoses(tex.pose, model.chunkCount);
    const meshes = createChunkMeshes(model, table, buildCullBuffers(model), sw, tex);
    const bonds = createBondLines(model);
    const physics = new RapierBackend({
      gravity: [0, -params.g, 0], impactThreshold: params.impactThreshold,
      solverIterations: params.solverIterations, pgsIterations: params.pgsIterations,
    });
    physics.createGround(0, ALL);
    const client = params.sync ? new InlineIntegrity(model, table) : new WorkerIntegrity(model, table);
    const sim = new DestructionSim(model, table, physics, { client, collisionBreak: { energyScale: params.impactEnergy, minSpeed: params.breakMinSpeed } });
    sim.dents = params.dents;
    scene.add(meshes.outer, meshes.inner, bonds.lines);
    cur = { model, table, meshes, bonds, broken: new Uint8Array(model.bondCount), physics, sim, shownBroken: 0, balls: [], skin: { adj, alive, sw, tex } };
    stats.mode = params.sync ? 'sync (inline)' : 'async (worker)';
    Object.assign(stats, {
      chunks: model.chunkCount, surface: surfaceChunkCount(model), bonds: model.bondCount,
      outerTris: meshes.cull.outerIndex.length / 3, innerTris: 0, brokenBonds: 0, islands: 0,
    });
    refreshColors(cur);
    applyVisibility();
  }

  /** Recolours bond lines from the broken flags. */
  function refreshColors(l: Loaded): void {
    updateBondColors(l.bonds, l.broken);
  }

  function applyVisibility(): void {
    if (!cur) return;
    cur.meshes.outer.visible = params.outer;
    cur.meshes.inner.visible = params.inner;
    cur.bonds.lines.visible = params.bondLines;
  }

  /** Scripted removal of random bonds (tier 0 drops whatever loses support). */
  function breakRandom(): void {
    if (!cur) return;
    const bonds = Array.from({ length: params.breakCount }, () => Math.floor(Math.random() * cur!.model.bondCount));
    cur.sim.removeBonds(bonds);
  }

  /** Reveals faces of newly broken bonds, re-weights the skin around them, writes island chunk poses, refreshes stats/colours. */
  function syncRender(l: Loaded): void {
    const { sim, meshes } = l;
    const { skin } = l;
    const first = l.shownBroken;
    for (; l.shownBroken < sim.brokenTotal; l.shownBroken++) {
      const b = sim.brokenLog[l.shownBroken];
      l.broken[b] = 1;
      skin.alive[b] = 0;
      revealBondFaces(meshes, b);
    }
    const changed = l.shownBroken > first;
    if (changed) {
      reweightBroken(skin.sw, l.model, skin.adj, skin.alive, sim.brokenLog.subarray(first, l.shownBroken));
      if (skin.sw.dirtyCount) {
        uploadDirtyWeights(skin.tex, skin.sw);
        writeSkinRanges(meshes.skinRange.array as Uint32Array, skin.sw);
        meshes.skinRange.needsUpdate = true;
      }
    }
    for (const isl of sim.handoff.islands) {
      if (isl.removed) continue;
      l.physics.readBodyPoses([isl.body], pose);
      writeIslandPoses(skin.tex.pose, isl.chunks, pose, 0, isl.local, l.model.chunkPos);
    }
    // demoted (frozen deformed) chunks: static pose written once
    const h = sim.handoff;
    for (const c of h.staticPoseDirty) {
      writeChunkPose(skin.tex.pose, c, h.setPos, 3 * c, h.setQ, 4 * c, l.model.chunkPos, 3 * c);
      // a dented `offsets` chunk moves off its neighbours with bonds intact: show its cut faces so the gaps aren't see-through
      if (l.table.materials[l.model.chunkMaterial[c]].deform === 'offsets') {
        for (let k = skin.adj.adjOffset[c]; k < skin.adj.adjOffset[c + 1]; k++) revealBondFaces(meshes, skin.adj.adjBond[k]);
      }
    }
    if (sim.handoff.islands.length || h.staticPoseDirty.length) skin.tex.poseTex.needsUpdate = true;
    h.staticPoseDirty.length = 0;
    if (changed) refreshColors(l);
    stats.innerTris = meshes.cull.innerCount / 3;
    stats.brokenBonds = sim.brokenTotal;
    stats.islands = sim.handoff.islands.length;
    stats.joints = sim.chains.aliveJoints; stats.brokenJoints = sim.chains.brokenJoints; stats.collisionBreaks = sim.breaker.broken;
    Object.assign(stats, { version: sim.version, replies: sim.replies, latencySteps: sim.replyLatencySteps });
  }

  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  /** Ray from the camera through the last pointer position (NDC). */
  function pointerRay(): THREE.Ray {
    raycaster.setFromCamera(ndc, camera);
    return raycaster.ray;
  }

  /** Chunk under the pointer that is still part of the static structure, or -1. */
  function pickStaticChunk(): number {
    if (!cur) return -1;
    pointerRay();
    const hits = raycaster.intersectObjects([cur.meshes.outer, cur.meshes.inner], false);
    const offs = cur.model.renderVertOffset;
    for (const h of hits) {
      if (!h.face) continue;
      // Vertex -> chunk: largest c with renderVertOffset[c] <= vertex (offsets are (n+1,), ascending).
      let lo = 0, hi = cur.model.chunkCount - 1;
      while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (offs[mid] <= h.face.a) lo = mid; else hi = mid - 1; }
      if (cur.sim.isStatic[lo]) return lo;
    }
    return -1;
  }

  /** Scripted removal of every bond of the picked chunk (it drops; dependents follow via tier 0). */
  function removePickedChunk(): void {
    const c = pickStaticChunk();
    if (c < 0 || !cur) return;
    const m = cur.model, bonds: number[] = [];
    for (let b = 0; b < m.bondCount; b++) if ((m.bondA[b] === c || m.bondB[b] === c) && !cur.broken[b]) bonds.push(b);
    cur.sim.removeBonds(bonds);
  }

  /** Throws a ball from the camera towards the pointer (aim raised to cancel the gravity drop at the hit distance). */
  function throwBall(): void {
    if (!cur) return;
    const ray = pointerRay();
    const hit = raycaster.intersectObjects([cur.meshes.outer, cur.meshes.inner], false)[0];
    const dist = hit ? hit.distance : 8;
    const t = dist / params.ballSpeed;
    const aim = ray.direction.clone().multiplyScalar(dist).add(new THREE.Vector3(0, 0.5 * 9.81 * t * t, 0)).normalize();
    const v = aim.multiplyScalar(params.ballSpeed);
    const p = ray.origin;
    const body = cur.physics.createSphere(params.ballRadius, [p.x, p.y, p.z], [v.x, v.y, v.z], 7850, ALL);
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(params.ballRadius, 24, 16), new THREE.MeshStandardNodeMaterial({ color: 0x444a52, metalness: 0.8, roughness: 0.3 }));
    mesh.castShadow = true;
    scene.add(mesh);
    cur.balls.push({ body, mesh });
  }

  const gui = new GUI({ title: 'destruction3d' });
  gui.add(params, 'scene', SCENES).onChange(load);
  const vf = gui.addFolder('debug view');
  vf.add(params, 'outer').name('outer faces').onChange(applyVisibility);
  vf.add(params, 'inner').name('inner faces').onChange(applyVisibility);
  vf.add(params, 'bondLines').name('bond lines').onChange(applyVisibility);

  const simf = gui.addFolder('sim');
  simf.add(params, 'sync').name('sync fracture (inline)').onChange(load);
  simf.add(params, 'paused');
  simf.add({ step: () => { if (cur) stepOnce(cur); } }, 'step').name('step once (paused)');
  simf.add(params, 'timeScale', 0.05, 2, 0.05).name('time scale');
  simf.add(params, 'physicsHz', 30, 240, 10).name('physics Hz');
  simf.add(params, 'substeps', 1, 8, 1);
  const rf = simf.addFolder('reload on change');
  rf.add(params, 'g', 0, 30, 0.01).name('gravity').onFinishChange(load);
  rf.add(params, 'impactThreshold', 0, 20000, 50).name('impact threshold N').onFinishChange(load);
  rf.add(params, 'solverIterations', 1, 32, 1).name('rapier solver iters').onFinishChange(load);
  rf.add(params, 'pgsIterations', 1, 8, 1).name('rapier PGS iters').onFinishChange(load);

  const mf = gui.addFolder('material multipliers (reload)');
  mf.add(matScale, 'strength', 0.1, 100).onFinishChange(load);
  mf.add(matScale, 'stiffness', 0.1, 10).onFinishChange(load);
  mf.add(matScale, 'ductility', 0.1, 10).name('break strain').onFinishChange(load);
  mf.add(matScale, 'damageRate', 0.01, 10).name('damage rate').onFinishChange(load);
  mf.add(matScale, 'density', 0.1, 10).onFinishChange(load);
  const pm = mf.addFolder('strength per material').close();
  for (const name of Object.keys(perStrength)) pm.add(perStrength, name, 0.1, 100).onFinishChange(load);
  mf.add({ reset: () => { Object.assign(matScale, UNIT_SCALE); for (const k in perStrength) perStrength[k] = 1; mf.controllersRecursive().forEach((c) => c.updateDisplay()); load(); } }, 'reset').name('reset multipliers');

  const af = gui.addFolder('actions');
  af.add(params, 'breakCount', 1, 200, 1);
  af.add({ breakRandom }, 'breakRandom').name('break random bonds');
  af.add(params, 'ballSpeed', 1, 60);
  af.add(params, 'ballRadius', 0.05, 1);
  af.add(params, 'impactEnergy', 0, 1000).name('collision break energy x').onFinishChange(load);
  af.add(params, 'breakMinSpeed', 0, 10, 0.1).name('break min speed m/s').onChange((v: number) => { if (cur) cur.sim.breaker.minSpeed = v; });
  af.add(params, 'dents').name('dents (fake offsets)').onChange((v: boolean) => { if (cur) cur.sim.dents = v; });
  af.add({ throwBall }, 'throwBall').name('throw ball (click / space)');
  af.add({ hint: 'shift+click: remove chunk' }, 'hint').name('tip').disable();
  af.add({ reset: load }, 'reset');
  const sf = gui.addFolder('stats');
  for (const k of Object.keys(stats) as (keyof typeof stats)[]) sf.add(stats, k).disable().listen();

  // Pointer: track for aiming; a click (no drag) throws, shift+click removes the chunk under the cursor.
  let downX = 0, downY = 0;
  renderer.domElement.addEventListener('pointermove', (e) => {
    ndc.set((e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
  });
  renderer.domElement.addEventListener('pointerdown', (e) => { downX = e.clientX; downY = e.clientY; });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (Math.hypot(e.clientX - downX, e.clientY - downY) > 4 || e.button !== 0) return; // orbit drag
    ndc.set((e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
    if (e.shiftKey) removePickedChunk(); else throwBall();
  });
  window.addEventListener('keydown', (e) => { if (e.code === 'Space') throwBall(); });
  window.addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  // Debug handle for automated browser checks (console: demo.cur.sim.brokenTotal, ...).
  (window as unknown as { demo: unknown }).demo = { get cur() { return cur; }, params, stats, matScale, load, throwBall, breakRandom, removePickedChunk, ndc };

  /** One fixed step: `substeps` physics + integrity ticks of dt/substeps (Rapier auto-drains events per step). */
  function stepOnce(l: Loaded): void {
    const h = 1 / params.physicsHz / params.substeps;
    const t0 = performance.now();
    for (let k = 0; k < params.substeps; k++) { l.physics.step(h); l.sim.tick(h); }
    simMs += performance.now() - t0;
  }
  let simMs = 0;

  const pose = new Float32Array(7);
  load();
  let acc = 0, last = performance.now();
  renderer.setAnimationLoop(() => {
    const now = performance.now();
    acc = params.paused ? 0 : Math.min(acc + params.timeScale * (now - last) / 1000, 0.1); // clamp: never spiral after a stall
    last = now;
    if (cur) {
      const dt = 1 / params.physicsHz;
      simMs = 0;
      while (acc >= dt) { stepOnce(cur); acc -= dt; }
      stats.simTickMs = simMs.toFixed(2);
      syncRender(cur);
      for (const b of cur.balls) {
        cur.physics.readBodyPoses([b.body], pose);
        b.mesh.position.set(pose[0], pose[1], pose[2]);
        b.mesh.quaternion.set(pose[3], pose[4], pose[5], pose[6]);
      }
    }
    controls.update();
    renderer.render(scene, camera);
  });
}

main().catch((e) => {
  console.error(e);
  document.body.innerHTML = `<pre style="color:#f88;padding:1em">${String(e?.stack ?? e)}</pre>`;
});
