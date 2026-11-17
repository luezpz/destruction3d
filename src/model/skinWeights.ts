// Variable-length, topology-aware skin weights in CSR layout (NOTES Design §4 "Weights", "Topology-aware", "Variable-length weights").
// Headless: no three.js/Rapier. Setup allocates; `reweightBroken` is allocation-free (scratch preallocated).

import { ChunkFlag, ChunkModel } from './ChunkModel';
import { BondAdjacency } from '../kernels/bondAdjacency';

/** Max representable weight of an entry (unorm16). */
export const W16 = 65535;

export interface SkinWeightOptions {
  /** Support radius factor: R_j = rho * r_j, r_j = bounding radius of chunk j's render verts. Must be > 1 so the owner always has phi > 0. */
  rho?: number;
  /** Weights below this (after normalizing) are dropped. */
  minWeight?: number;
  /** Soft cap on influences per vertex (largest kept). */
  maxInfluences?: number;
}

/**
 * Skin weights for one ChunkModel mesh. Entries are `chunkId << 16 | w16` (unorm16 weight); vertex v uses
 * entries[vertOffset[v] .. vertOffset[v] + vertCount[v]) and its slot holds vertCapacity[v] entries (initial count; counts only shrink).
 */
export interface SkinWeights {
  vertexCount: number;
  vertOffset: Uint32Array; // (V,)
  vertCount: Uint32Array; // (V,)
  vertCapacity: Uint32Array; // (V,)
  entries: Uint32Array; // (sum capacity,)
  vertOwner: Uint32Array; // (V,) chunk owning the vertex
  /** Rest positions in the structure frame (chunkPos + renderPos): (3V,). */
  restPos: Float64Array;
  /** Support radius R_j: (n,). */
  chunkRadius: Float64Array;
  /** Chunk -> vertices it initially influences, CSR (also the hull input V_c of NOTES Design §5): verts of j are chunkVerts[chunkVertOffset[j] .. [j+1]). */
  chunkVertOffset: Uint32Array; // (n+1,)
  chunkVerts: Uint32Array;
  /** Vertices rewritten by the last `reweightBroken` call: dirtyVerts[0 .. dirtyCount). Upload only these. */
  dirtyVerts: Uint32Array;
  dirtyCount: number;
  // scratch (alloc-free reweighting)
  readonly maxInfluences: number;
  readonly minWeight: number;
  /** @internal per-chunk stamps for candidate set / BFS reach / dirty marks. */
  candStamp: Uint32Array;
  reachStamp: Uint32Array;
  dirtyStamp: Uint32Array; // (V,)
  stamp: number;
  cand: Uint32Array; // (n,)
  queue: Uint32Array; // (n,)
  wTmp: Float64Array; // (n,)
  outEntries: Uint32Array; // (maxInfluences,)
}

/** Compact kernel phi(s) = (1 - s^2)^3 for s < 1, else 0. @param s normalized distance |p - c_j| / R_j. */
export function kernelPhi(s: number): number {
  if (s >= 1) return 0;
  const t = 1 - s * s;
  return t * t * t;
}

/**
 * Builds weights for every render vertex. Non-SKINNED owners get one entry (rigid, count == 1 fast path); SKINNED owners blend
 * SKINNED chunks within their support that are reachable from the owner through alive bonds inside the support.
 * @param model mesh + bonds; @param adj chunk -> bond adjacency; @param alive (m,) 1 = intact bond; @returns weights (setup-time, allocates).
 */
export function buildSkinWeights(model: ChunkModel, adj: BondAdjacency, alive: Uint8Array, opts: SkinWeightOptions = {}): SkinWeights {
  const rho = opts.rho ?? 1.6, minWeight = opts.minWeight ?? 1e-3, maxInfluences = opts.maxInfluences ?? 32;
  if (!(rho > 1)) throw new Error('skinWeights: rho must be > 1');
  const n = model.chunkCount, V = model.renderVertOffset[n];
  const { chunkPos, renderPos, renderVertOffset } = model;

  const vertOwner = new Uint32Array(V), restPos = new Float64Array(3 * V), chunkRadius = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    let r2 = 0;
    for (let v = renderVertOffset[j]; v < renderVertOffset[j + 1]; v++) {
      vertOwner[v] = j;
      for (let k = 0; k < 3; k++) restPos[3 * v + k] = chunkPos[3 * j + k] + renderPos[3 * v + k];
      r2 = Math.max(r2, renderPos[3 * v] ** 2 + renderPos[3 * v + 1] ** 2 + renderPos[3 * v + 2] ** 2);
    }
    const r = r2 > 0 ? Math.sqrt(r2) : Math.cbrt(model.chunkVolume[j]) * 0.5;
    chunkRadius[j] = rho * r;
  }

  const sw: SkinWeights = {
    vertexCount: V, vertOffset: new Uint32Array(V), vertCount: new Uint32Array(V), vertCapacity: new Uint32Array(V),
    entries: new Uint32Array(0), vertOwner, restPos, chunkRadius,
    chunkVertOffset: new Uint32Array(n + 1), chunkVerts: new Uint32Array(0),
    dirtyVerts: new Uint32Array(V), dirtyCount: 0, maxInfluences, minWeight,
    candStamp: new Uint32Array(n), reachStamp: new Uint32Array(n), dirtyStamp: new Uint32Array(V), stamp: 0,
    cand: new Uint32Array(n), queue: new Uint32Array(n), wTmp: new Float64Array(n), outEntries: new Uint32Array(maxInfluences),
  };

  // Uniform grid over SKINNED chunk centres, cell = max support radius, so a query touches 27 cells.
  let cell = 0;
  for (let j = 0; j < n; j++) if (model.chunkFlags[j] & ChunkFlag.SKINNED) cell = Math.max(cell, chunkRadius[j]);
  const grid = new Map<string, number[]>();
  const key = (x: number, y: number, z: number) => `${x},${y},${z}`;
  for (let j = 0; j < n; j++) {
    if (!(model.chunkFlags[j] & ChunkFlag.SKINNED)) continue;
    const k = key(Math.floor(chunkPos[3 * j] / cell), Math.floor(chunkPos[3 * j + 1] / cell), Math.floor(chunkPos[3 * j + 2] / cell));
    let list = grid.get(k);
    if (!list) grid.set(k, (list = []));
    list.push(j);
  }

  const all: number[] = [];
  for (let v = 0; v < V; v++) {
    const owner = vertOwner[v];
    sw.vertOffset[v] = all.length;
    let count: number;
    if (!(model.chunkFlags[owner] & ChunkFlag.SKINNED)) {
      sw.outEntries[0] = (owner << 16) | W16;
      count = 1;
    } else {
      const px = restPos[3 * v], py = restPos[3 * v + 1], pz = restPos[3 * v + 2];
      const gx = Math.floor(px / cell), gy = Math.floor(py / cell), gz = Math.floor(pz / cell);
      let nc = 0;
      for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
        const list = grid.get(key(gx + dx, gy + dy, gz + dz));
        if (list) for (const j of list) if (j !== owner && supportPhi(sw, model, v, j) > 0) sw.cand[nc++] = j;
      }
      sw.cand[nc++] = owner;
      count = weighVertex(sw, model, adj, alive, v, nc);
    }
    for (let k = 0; k < count; k++) all.push(sw.outEntries[k]);
    sw.vertCount[v] = sw.vertCapacity[v] = count;
  }
  sw.entries = Uint32Array.from(all);

  // Chunk -> influenced vertices.
  const cvo = sw.chunkVertOffset;
  for (let e = 0; e < all.length; e++) cvo[(all[e] >>> 16) + 1]++;
  for (let j = 0; j < n; j++) cvo[j + 1] += cvo[j];
  const fill = cvo.slice(0, n), chunkVerts = new Uint32Array(all.length);
  for (let v = 0; v < V; v++) for (let e = sw.vertOffset[v]; e < sw.vertOffset[v] + sw.vertCount[v]; e++) chunkVerts[fill[sw.entries[e] >>> 16]++] = v;
  sw.chunkVerts = chunkVerts;
  return sw;
}

/** phi(|p_v - c_j| / R_j) for rest vertex v and chunk j. */
function supportPhi(sw: SkinWeights, model: ChunkModel, v: number, j: number): number {
  const dx = sw.restPos[3 * v] - model.chunkPos[3 * j], dy = sw.restPos[3 * v + 1] - model.chunkPos[3 * j + 1], dz = sw.restPos[3 * v + 2] - model.chunkPos[3 * j + 2];
  return kernelPhi(Math.sqrt(dx * dx + dy * dy + dz * dz) / sw.chunkRadius[j]);
}

/**
 * Weighs vertex v over candidates sw.cand[0..nc) (owner included): keeps those reachable from the owner via alive bonds
 * between candidates, normalizes phi, drops < minWeight, caps (largest kept), renormalizes and quantizes to unorm16 summing to W16 exactly.
 * Writes sw.outEntries sorted by weight desc (ties: id asc, so seam copies match) and returns the count. Alloc-free.
 */
function weighVertex(sw: SkinWeights, model: ChunkModel, adj: BondAdjacency, alive: Uint8Array, v: number, nc: number): number {
  const s = ++sw.stamp, owner = sw.vertOwner[v];
  for (let k = 0; k < nc; k++) sw.candStamp[sw.cand[k]] = s;
  // BFS from owner restricted to the candidate set; queue doubles as the reached list.
  const q = sw.queue;
  let head = 0, tail = 0;
  q[tail++] = owner; sw.reachStamp[owner] = s;
  while (head < tail) {
    const c = q[head++];
    for (let i = adj.adjOffset[c]; i < adj.adjOffset[c + 1]; i++) {
      const b = adj.adjBond[i];
      if (!alive[b]) continue;
      const o = model.bondA[b] === c ? model.bondB[b] : model.bondA[b];
      if (sw.candStamp[o] === s && sw.reachStamp[o] !== s) { sw.reachStamp[o] = s; q[tail++] = o; }
    }
  }
  // phi, normalize, sort desc (insertion sort; lists are short).
  const w = sw.wTmp;
  let sum = 0;
  for (let k = 0; k < tail; k++) sum += (w[k] = supportPhi(sw, model, v, q[k]));
  for (let k = 1; k < tail; k++) {
    const wk = w[k], ik = q[k];
    let i = k - 1;
    while (i >= 0 && (w[i] < wk || (w[i] === wk && q[i] > ik))) { w[i + 1] = w[i]; q[i + 1] = q[i]; i--; }
    w[i + 1] = wk; q[i + 1] = ik;
  }
  let count = Math.min(tail, sw.maxInfluences);
  while (count > 1 && w[count - 1] / sum < sw.minWeight) count--;
  let kept = 0;
  for (let k = 0; k < count; k++) kept += w[k];
  let total = 0;
  for (let k = 0; k < count; k++) total += (w[k] = Math.round((w[k] / kept) * W16));
  w[0] += W16 - total; // residual to the largest: exact partition of unity in unorm16
  for (let k = 0; k < count; k++) sw.outEntries[k] = ((q[k] << 16) | w[k]) >>> 0;
  return count;
}

/**
 * Re-weights, in place, every vertex influenced by an end of a broken bond (NOTES Design §4: breaks only remove influences, so the new
 * count <= capacity). Candidates are the vertex's current influences. Fills sw.dirtyVerts/dirtyCount. Alloc-free.
 * @param bonds broken bond ids (already marked dead in `alive`).
 */
export function reweightBroken(sw: SkinWeights, model: ChunkModel, adj: BondAdjacency, alive: Uint8Array, bonds: ArrayLike<number>): void {
  sw.dirtyCount = 0;
  const ds = ++sw.stamp;
  for (let i = 0; i < bonds.length; i++) {
    for (let end = 0; end < 2; end++) {
      const j = end === 0 ? model.bondA[bonds[i]] : model.bondB[bonds[i]];
      for (let p = sw.chunkVertOffset[j]; p < sw.chunkVertOffset[j + 1]; p++) {
        const v = sw.chunkVerts[p];
        if (sw.dirtyStamp[v] === ds || sw.vertCount[v] <= 1) continue;
        sw.dirtyStamp[v] = ds;
        const off = sw.vertOffset[v], nc = sw.vertCount[v];
        for (let k = 0; k < nc; k++) sw.cand[k] = sw.entries[off + k] >>> 16;
        const count = weighVertex(sw, model, adj, alive, v, nc);
        for (let k = 0; k < count; k++) sw.entries[off + k] = sw.outEntries[k];
        sw.vertCount[v] = count;
        sw.dirtyVerts[sw.dirtyCount++] = v;
      }
    }
  }
}
