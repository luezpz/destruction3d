// Debug app: generated structure, culled outer/inner render, bond lines, Rapier, and the M3 core loop (DestructionSim):
// impacts -> stress/damage -> breaks -> falling islands. Islands are placed on the CPU (placeChunk) until M4 skinning.

import * as THREE from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import GUI from 'lil-gui';
import { ChunkModel, surfaceChunkCount } from './model/ChunkModel';
import { createMaterialTable } from './model/Material';
import { buildCullBuffers } from './render/cullBuffers';
import { ChunkMeshes, createChunkMeshes, revealBondFaces } from './render/chunkMesh';
import { BondLines, createBondLines, updateBondColors } from './render/bondLines';
import { packGroups } from './physics/PhysicsBackend';
import { RapierBackend, initRapier } from './physics/RapierBackend';
import { DestructionSim } from './sim/DestructionSim';
import { WorkerIntegrity } from './integrity/IntegrityClient';
import { placeChunk } from './render/islandPose';
import { SCENES, SceneName, buildScene } from './scenes';

const FIXED_DT = 1 / 60;
const ALL = packGroups(0xffff, 0xffff);

/** Everything that belongs to the currently loaded structure. */
interface Loaded {
  model: ChunkModel;
  meshes: ChunkMeshes;
  bonds: BondLines;
  broken: Uint8Array;
  physics: RapierBackend;
  sim: DestructionSim;
  /** brokenLog entries already revealed/coloured. */
  shownBroken: number;
  balls: { body: number; mesh: THREE.Mesh }[];
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

  const table = createMaterialTable();
  const params = { scene: SCENES[0] as SceneName, outer: true, inner: true, bondLines: false, breakCount: 10, ballSpeed: 15, ballRadius: 0.25 };
  const stats = { chunks: 0, surface: 0, bonds: 0, outerTris: 0, innerTris: 0, brokenBonds: 0, islands: 0 };
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
    const model = buildScene(params.scene, table);
    const meshes = createChunkMeshes(model, table, buildCullBuffers(model));
    const bonds = createBondLines(model);
    const physics = new RapierBackend({ impactThreshold: 500 });
    physics.createGround(0, ALL);
    const sim = new DestructionSim(model, table, physics, { client: new WorkerIntegrity(model, table) });
    (meshes.outer.geometry.getAttribute('position') as THREE.BufferAttribute).setUsage(THREE.DynamicDrawUsage);
    scene.add(meshes.outer, meshes.inner, bonds.lines);
    cur = { model, meshes, bonds, broken: new Uint8Array(model.bondCount), physics, sim, shownBroken: 0, balls: [] };
    Object.assign(stats, {
      chunks: model.chunkCount, surface: surfaceChunkCount(model), bonds: model.bondCount,
      outerTris: meshes.cull.outerIndex.length / 3, innerTris: 0, brokenBonds: 0, islands: 0,
    });
    updateBondColors(bonds, sim.ratio, cur.broken);
    applyVisibility();
  }

  function applyVisibility(): void {
    if (!cur) return;
    cur.meshes.outer.visible = params.outer;
    cur.meshes.inner.visible = params.inner;
    cur.bonds.lines.visible = params.bondLines;
  }

  /** Scripted removal of random bonds (tier 0 drops whatever loses support; tier 1 handles the overload). */
  function breakRandom(): void {
    if (!cur) return;
    const bonds = Array.from({ length: params.breakCount }, () => Math.floor(Math.random() * cur!.model.bondCount));
    cur.sim.removeBonds(bonds);
  }

  /** Reveals faces of newly broken bonds, places island chunks from body poses, refreshes stats/colours. */
  function syncRender(l: Loaded): void {
    const { sim, meshes } = l;
    let changed = false;
    for (; l.shownBroken < sim.brokenTotal; l.shownBroken++) {
      const b = sim.brokenLog[l.shownBroken];
      l.broken[b] = 1;
      revealBondFaces(meshes, b);
      changed = true;
    }
    const posAttr = meshes.outer.geometry.getAttribute('position') as THREE.BufferAttribute;
    const nrmAttr = meshes.outer.geometry.getAttribute('normal') as THREE.BufferAttribute;
    for (const isl of sim.handoff.islands) {
      l.physics.readBodyPoses([isl.body], pose);
      for (const c of isl.chunks) placeChunk(l.model, c, pose, 0, isl.com, posAttr.array as Float32Array, nrmAttr.array as Float32Array);
    }
    if (sim.handoff.islands.length) {
      posAttr.needsUpdate = true; nrmAttr.needsUpdate = true;
      for (const m of [meshes.outer, meshes.inner]) m.geometry.computeBoundingSphere();
    }
    if (changed || frame % 10 === 0) updateBondColors(l.bonds, sim.ratio, l.broken);
    stats.innerTris = meshes.cull.innerCount / 3;
    stats.brokenBonds = sim.brokenTotal;
    stats.islands = sim.handoff.islands.length;
  }

  /** Throws a ball from the camera along the view direction. */
  function throwBall(): void {
    if (!cur) return;
    const dir = new THREE.Vector3();
    camera.getWorldDirection(dir);
    const v = dir.clone().multiplyScalar(params.ballSpeed);
    const p = camera.position;
    const body = cur.physics.createSphere(params.ballRadius, [p.x, p.y, p.z], [v.x, v.y, v.z], 7850, ALL);
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(params.ballRadius, 24, 16), new THREE.MeshStandardNodeMaterial({ color: 0x444a52, metalness: 0.8, roughness: 0.3 }));
    mesh.castShadow = true;
    scene.add(mesh);
    cur.balls.push({ body, mesh });
  }

  const gui = new GUI({ title: 'destruction3d (M3)' });
  gui.add(params, 'scene', SCENES).onChange(load);
  gui.add(params, 'outer').name('outer faces').onChange(applyVisibility);
  gui.add(params, 'inner').name('inner faces').onChange(applyVisibility);
  gui.add(params, 'bondLines').name('bond lines').onChange(applyVisibility);
  gui.add(params, 'breakCount', 1, 200, 1);
  gui.add({ breakRandom }, 'breakRandom').name('break random bonds');
  gui.add(params, 'ballSpeed', 1, 60);
  gui.add(params, 'ballRadius', 0.05, 1);
  gui.add({ throwBall }, 'throwBall').name('throw ball (space)');
  gui.add({ reset: load }, 'reset');
  const sf = gui.addFolder('stats');
  for (const k of Object.keys(stats) as (keyof typeof stats)[]) sf.add(stats, k).disable().listen();

  window.addEventListener('keydown', (e) => { if (e.code === 'Space') throwBall(); });
  window.addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  const pose = new Float32Array(7);
  let frame = 0;
  load();
  let acc = 0, last = performance.now();
  renderer.setAnimationLoop(() => {
    const now = performance.now();
    acc = Math.min(acc + (now - last) / 1000, 0.1); // clamp: never spiral after a stall
    last = now;
    if (cur) {
      while (acc >= FIXED_DT) {
        cur.physics.step(FIXED_DT);
        cur.sim.tick(FIXED_DT);
        acc -= FIXED_DT;
      }
      frame++;
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
