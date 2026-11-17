// Skin weights (CSR, topology-aware, in-place shrink) and CPU skin reference (NOTES Design §4 "Tests").
import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { buildBondAdjacency } from '../src/kernels/bondAdjacency';
import { buildSkinWeights, reweightBroken, SkinWeights, W16 } from '../src/model/skinWeights';
import { POSE_STRIDE, skinVertices, writeChunkPose, writeIdentityPoses } from '../src/kernels/skin';

const table = createMaterialTable();
const steel = materialId(table, 'steel');
const concrete = materialId(table, 'concrete');

/** nx x nz steel sheet of unit cubes in the y=0 layer, plus one concrete cube beyond x = nx. */
function sheet(nx: number, nz: number) {
  const boxes: Box[] = [];
  for (let x = 0; x < nx; x++) for (let z = 0; z < nz; z++) boxes.push({ min: [x, 0, z], max: [x + 1, 1, z + 1], material: steel, support: false });
  boxes.push({ min: [nx, 0, 0], max: [nx + 1, 1, 1], material: concrete, support: false });
  const model = generateFromBoxes(boxes, table);
  const adj = buildBondAdjacency(model.chunkCount, model.bondA, model.bondB, model.bondCount);
  const alive = new Uint8Array(model.bondCount).fill(1);
  return { model, adj, alive, sw: buildSkinWeights(model, adj, alive) };
}

function influences(sw: SkinWeights, v: number): Map<number, number> {
  const m = new Map<number, number>();
  for (let e = sw.vertOffset[v]; e < sw.vertOffset[v] + sw.vertCount[v]; e++) m.set(sw.entries[e] >>> 16, sw.entries[e] & 0xffff);
  return m;
}

function bondBetween(model: ReturnType<typeof sheet>['model'], p: number, q: number) {
  for (let b = 0; b < model.bondCount; b++) if ((model.bondA[b] === p && model.bondB[b] === q) || (model.bondA[b] === q && model.bondB[b] === p)) return b;
  throw new Error('no bond');
}

describe('skin weights', () => {
  it('CSR slots are consistent, partition of unity, non-skinned count == 1', () => {
    const { model, sw } = sheet(4, 4);
    let next = 0, maxCount = 0;
    for (let v = 0; v < sw.vertexCount; v++) {
      expect(sw.vertOffset[v]).toBe(next);
      next += sw.vertCapacity[v];
      let sum = 0;
      for (const w of influences(sw, v).values()) sum += w;
      expect(sum).toBe(W16);
      maxCount = Math.max(maxCount, sw.vertCount[v]);
      if (model.chunkMaterial[sw.vertOwner[v]] === concrete) expect(sw.vertCount[v]).toBe(1);
      for (const id of influences(sw, v).keys()) if (id !== sw.vertOwner[v]) expect(model.chunkMaterial[id]).toBe(steel);
    }
    expect(next).toBe(sw.entries.length);
    expect(maxCount).toBeGreaterThan(1);
    expect(maxCount).toBeLessThanOrEqual(32);
  });

  it('seam copies are identical while intact and disjoint after a break; shrink stays in slot', () => {
    const { model, adj, alive, sw } = sheet(2, 1);
    const b = bondBetween(model, 0, 1);
    const seam: [number, number][] = [];
    for (let v = model.renderVertOffset[0]; v < model.renderVertOffset[1]; v++)
      for (let u = model.renderVertOffset[1]; u < model.renderVertOffset[2]; u++)
        if ([0, 1, 2].every((k) => Math.abs(sw.restPos[3 * v + k] - sw.restPos[3 * u + k]) < 1e-12)) seam.push([v, u]);
    expect(seam.length).toBeGreaterThan(0);
    for (const [v, u] of seam) expect([...influences(sw, v)]).toEqual([...influences(sw, u)]);

    const before = sw.entries.slice(), cap = sw.vertCapacity.slice();
    alive[b] = 0;
    reweightBroken(sw, model, adj, alive, [b]);
    expect(sw.dirtyCount).toBeGreaterThan(0);
    for (const [v, u] of seam) {
      expect([...influences(sw, v).keys()]).toEqual([0]);
      expect([...influences(sw, u).keys()]).toEqual([1]);
    }
    for (let v = 0; v < sw.vertexCount; v++) expect(sw.vertCount[v]).toBeLessThanOrEqual(cap[v]);
    // entries beyond each vertex's new count are untouched (no overrun into the next slot)
    for (let v = 0; v < sw.vertexCount; v++) {
      const end = sw.vertOffset[v] + sw.vertCapacity[v];
      for (let e = sw.vertOffset[v] + sw.vertCount[v]; e < end; e++) expect(sw.entries[e]).toBe(before[e]);
    }
  });

  it('a redundant break keeps blending through the remaining path', () => {
    const { model, adj, alive, sw } = sheet(2, 2); // chunks 0:(0,0) 1:(0,1) 2:(1,0) 3:(1,1)
    // centre vertex (1,*,1) touches all four; break 0-2, still reachable via 0-1-3-2
    const b = bondBetween(model, 0, 2);
    let v0 = -1;
    for (let v = model.renderVertOffset[0]; v < model.renderVertOffset[1]; v++)
      if (Math.abs(sw.restPos[3 * v] - 1) < 1e-12 && Math.abs(sw.restPos[3 * v + 2] - 1) < 1e-12) { v0 = v; break; }
    expect(influences(sw, v0).size).toBe(4);
    alive[b] = 0;
    reweightBroken(sw, model, adj, alive, [b]);
    expect(influences(sw, v0).size).toBe(4);
  });
});

describe('CPU skin', () => {
  it('rest pose is identity', () => {
    const { model, sw } = sheet(3, 3);
    const pose = new Float64Array(POSE_STRIDE * model.chunkCount);
    writeIdentityPoses(pose, model.chunkCount);
    const p = new Float64Array(3 * sw.vertexCount), n = new Float64Array(3 * sw.vertexCount);
    skinVertices(sw.entries, sw.vertOffset, sw.vertCount, sw.restPos, model.renderNormal, pose, p, n, 0, sw.vertexCount);
    for (let i = 0; i < p.length; i++) {
      expect(p[i]).toBeCloseTo(sw.restPos[i], 12);
      expect(n[i]).toBeCloseTo(model.renderNormal[i], 12);
    }
  });

  it('rigid motion of all chunks moves every vertex rigidly; bent sheet normals stay unit', () => {
    const { model, sw } = sheet(4, 2);
    const N = model.chunkCount, pose = new Float64Array(POSE_STRIDE * N);
    const p = new Float64Array(3 * sw.vertexCount), n = new Float64Array(3 * sw.vertexCount);
    // rigid: rotate 0.7 rad about y, translate (1,2,3)
    const h = 0.35, q = [0, Math.sin(h), 0, Math.cos(h)];
    for (let i = 0; i < N; i++) {
      const c = model.chunkPos, cx = c[3 * i], cz = c[3 * i + 2];
      const x = [Math.cos(0.7) * cx + Math.sin(0.7) * cz + 1, c[3 * i + 1] + 2, -Math.sin(0.7) * cx + Math.cos(0.7) * cz + 3];
      writeChunkPose(pose, i, x, 0, q, 0, c, 3 * i);
    }
    skinVertices(sw.entries, sw.vertOffset, sw.vertCount, sw.restPos, model.renderNormal, pose, p, n, 0, sw.vertexCount);
    for (let v = 0; v < sw.vertexCount; v++) {
      const rx = sw.restPos[3 * v], rz = sw.restPos[3 * v + 2];
      expect(p[3 * v]).toBeCloseTo(Math.cos(0.7) * rx + Math.sin(0.7) * rz + 1, 4); // unorm16 weights: ~1e-5 error
      expect(p[3 * v + 2]).toBeCloseTo(-Math.sin(0.7) * rx + Math.cos(0.7) * rz + 3, 4);
    }
    // bend: chunk at column x rotated by 0.3 x about z, about the sheet's left edge
    for (let i = 0; i < N; i++) {
      const c = model.chunkPos, a = 0.3 * c[3 * i], qb = [0, 0, Math.sin(a / 2), Math.cos(a / 2)];
      const x = [Math.cos(a) * c[3 * i] - Math.sin(a) * c[3 * i + 1], Math.sin(a) * c[3 * i] + Math.cos(a) * c[3 * i + 1], c[3 * i + 2]];
      writeChunkPose(pose, i, x, 0, qb, 0, c, 3 * i);
    }
    skinVertices(sw.entries, sw.vertOffset, sw.vertCount, sw.restPos, model.renderNormal, pose, p, n, 0, sw.vertexCount);
    for (let v = 0; v < sw.vertexCount; v++) expect(Math.hypot(n[3 * v], n[3 * v + 1], n[3 * v + 2])).toBeCloseTo(1, 12);
  });
});
