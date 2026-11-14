// M1 debug app: generated structure, culled outer/inner render, bond lines, Rapier static colliders, ball drops with impact events.
// No integrity solver yet (M2/M3): "break random bonds" only exercises the reveal path.

import * as THREE from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import GUI from 'lil-gui';
import { ChunkModel, surfaceChunkCount } from './model/ChunkModel';
import { createMaterialTable } from './model/Material';
import { buildCullBuffers } from './render/cullBuffers';
import { ChunkMeshes, createChunkMeshes, revealBondFaces } from './render/chunkMesh';
import { BondLines, createBondLines, updateBondColors } from './render/bondLines';
import { GroupBit, packGroups } from './physics/PhysicsBackend';
import { RapierBackend, initRapier } from './physics/RapierBackend';
import { chunkHull } from './physics/fromModel';
import { SCENES, SceneName, buildScene } from './scenes';

const FIXED_DT = 1 / 60;
const ALL = packGroups(0xffff, 0xffff);
const STRUCTURE = packGroups(GroupBit.STRUCTURE, 0xffff);

/** Everything that belongs to the currently loaded structure. */
interface Loaded {
  model: ChunkModel;
  meshes: ChunkMeshes;
  bonds: BondLines;
  ratio: Float32Array;
  broken: Uint8Array;
  physics: RapierBackend;
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
  const stats = { chunks: 0, surface: 0, bonds: 0, outerTris: 0, innerTris: 0, impacts: 0, maxImpactN: 0 };
  let cur: Loaded | null = null;

  /** Disposes the current structure and loads the selected scene. */
  function load(): void {
    if (cur) {
      for (const o of [cur.meshes.outer, cur.meshes.inner, cur.bonds.lines, ...cur.balls.map((b) => b.mesh)]) {
        scene.remove(o);
        o.geometry.dispose();
      }
      cur.physics.dispose();
    }
    const model = buildScene(params.scene, table);
    const meshes = createChunkMeshes(model, table, buildCullBuffers(model));
    const bonds = createBondLines(model);
    const physics = new RapierBackend({ impactThreshold: 500 });
    physics.createGround(0, ALL);
    const structure = physics.createStaticBody();
    for (let i = 0; i < model.chunkCount; i++) physics.createStaticChunk(structure, chunkHull(model, i), STRUCTURE);
    scene.add(meshes.outer, meshes.inner, bonds.lines);
    cur = { model, meshes, bonds, ratio: new Float32Array(model.bondCount), broken: new Uint8Array(model.bondCount), physics, balls: [] };
    Object.assign(stats, {
      chunks: model.chunkCount, surface: surfaceChunkCount(model), bonds: model.bondCount,
      outerTris: meshes.cull.outerIndex.length / 3, innerTris: 0, impacts: 0, maxImpactN: 0,
    });
    updateBondColors(bonds, cur.ratio, cur.broken);
    applyVisibility();
  }

  function applyVisibility(): void {
    if (!cur) return;
    cur.meshes.outer.visible = params.outer;
    cur.meshes.inner.visible = params.inner;
    cur.bonds.lines.visible = params.bondLines;
  }

  /** Marks random intact bonds broken and reveals their faces (exercises the append-only inner buffer). */
  function breakRandom(): void {
    if (!cur) return;
    for (let k = 0; k < params.breakCount; k++) {
      const b = Math.floor(Math.random() * cur.model.bondCount);
      cur.broken[b] = 1;
      revealBondFaces(cur.meshes, b);
    }
    stats.innerTris = cur.meshes.cull.innerCount / 3;
    updateBondColors(cur.bonds, cur.ratio, cur.broken);
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

  const gui = new GUI({ title: 'destruction3d (M1)' });
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

  load();
  const pose = new Float32Array(7);
  let acc = 0, last = performance.now();
  renderer.setAnimationLoop(() => {
    const now = performance.now();
    acc = Math.min(acc + (now - last) / 1000, 0.1); // clamp: never spiral after a stall
    last = now;
    if (cur) {
      while (acc >= FIXED_DT) {
        cur.physics.step(FIXED_DT);
        cur.physics.drainImpactEvents((e) => {
          if (e.chunkA < 0 && e.chunkB < 0) return;
          stats.impacts++;
          stats.maxImpactN = Math.max(stats.maxImpactN, Math.round(e.force));
        });
        acc -= FIXED_DT;
      }
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
