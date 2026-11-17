// Builds a ChunkModel from axis-aligned boxes: bonds from face overlap, face tags (outer | inner+bondId), surface flags.
// Used by the grid and brick generators; Voronoi/glTF generators will produce the same ChunkModel later.

import { ChunkFlag, ChunkModel, NO_REINFORCEMENT, OUTER_FACE } from './ChunkModel';
import { MaterialTable, interfaceMaterial, materialId } from './Material';

export interface Box {
  min: [number, number, number];
  max: [number, number, number];
  material: number;
  support: boolean;
}

export interface BoxGenOptions {
  /** Touching tolerance (m) for face contact. */
  tol?: number;
  /** Explicit interface material for bonds between two boxes (e.g. mortar between bricks); null = same/derived. */
  interfaceFor?: (a: number, b: number) => number | null;
}

interface RawBond { a: number; b: number; axis: number; plane: number; lo: [number, number]; hi: [number, number] }

/** Finds touching box pairs with positive face overlap. Sweep along x. @returns bonds with a = lower side along `axis`. */
function findContacts(boxes: Box[], tol: number): RawBond[] {
  const order = boxes.map((_, i) => i).sort((i, j) => boxes[i].min[0] - boxes[j].min[0]);
  const out: RawBond[] = [];
  for (let oi = 0; oi < order.length; oi++) {
    const p = boxes[order[oi]];
    for (let oj = oi + 1; oj < order.length; oj++) {
      const q = boxes[order[oj]];
      if (q.min[0] > p.max[0] + tol) break;
      for (let k = 0; k < 3; k++) {
        let a: number, b: number;
        if (Math.abs(p.max[k] - q.min[k]) <= tol) { a = order[oi]; b = order[oj]; }
        else if (Math.abs(q.max[k] - p.min[k]) <= tol) { a = order[oj]; b = order[oi]; }
        else continue;
        const k1 = (k + 1) % 3, k2 = (k + 2) % 3;
        const lo: [number, number] = [Math.max(p.min[k1], q.min[k1]), Math.max(p.min[k2], q.min[k2])];
        const hi: [number, number] = [Math.min(p.max[k1], q.max[k1]), Math.min(p.max[k2], q.max[k2])];
        if (hi[0] - lo[0] > tol && hi[1] - lo[1] > tol) {
          out.push({ a, b, axis: k, plane: 0.5 * (boxes[a].max[k] + boxes[b].min[k]), lo, hi });
        }
      }
    }
  }
  out.sort((x, y) => x.a - y.a || x.b - y.b); // deterministic bond ids
  return out;
}

/** Sorted unique values with near-duplicates (within tol) merged. */
function uniqueCuts(vals: number[], tol: number): number[] {
  vals.sort((x, y) => x - y);
  const out: number[] = [];
  for (const v of vals) if (!out.length || v - out[out.length - 1] > tol) out.push(v);
  return out;
}

/**
 * Generates the ChunkModel. Mesh faces are split along overlap rectangles so each render triangle is either outer
 * or the cut face of exactly one bond (needed by the culled inner index buffer, NOTES Design §6).
 * @param boxes chunk boxes; @param table material table (derived interface materials may be appended).
 */
export function generateFromBoxes(boxes: Box[], table: MaterialTable, opts: BoxGenOptions = {}): ChunkModel {
  const tol = opts.tol ?? 1e-6;
  const n = boxes.length;
  if (n > 0xffff) throw new Error('more than 65535 chunks per mesh: split the mesh (16-bit chunk ids, NOTES Design §4)');

  const chunkPos = new Float64Array(3 * n), chunkVolume = new Float64Array(n);
  const chunkMaterial = new Uint16Array(n), chunkFlags = new Uint8Array(n);
  const hullPointOffset = new Uint32Array(n + 1), hullPoints = new Float64Array(n * 8 * 3);
  const hullTriOffset = new Uint32Array(n + 1), hullTris = new Uint32Array(n * 12 * 3);

  // Box corner c = (bit0 -> x, bit1 -> y, bit2 -> z); 12 outward CCW triangles.
  const BOX_TRIS = [0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5];
  for (let i = 0; i < n; i++) {
    const { min, max, material, support } = boxes[i];
    const h = [0.5 * (max[0] - min[0]), 0.5 * (max[1] - min[1]), 0.5 * (max[2] - min[2])];
    if (!(h[0] > 0 && h[1] > 0 && h[2] > 0)) throw new Error(`box ${i} has non-positive size`);
    for (let k = 0; k < 3; k++) chunkPos[3 * i + k] = 0.5 * (min[k] + max[k]);
    chunkVolume[i] = 8 * h[0] * h[1] * h[2];
    chunkMaterial[i] = material;
    const m = table.materials[material];
    if (!m) throw new Error(`box ${i}: unknown material id ${material}`);
    chunkFlags[i] = (support ? ChunkFlag.SUPPORT : 0) | (m.deform === 'joints' ? ChunkFlag.SKINNED : 0);
    for (let c = 0; c < 8; c++) {
      hullPoints[24 * i + 3 * c] = c & 1 ? h[0] : -h[0];
      hullPoints[24 * i + 3 * c + 1] = c & 2 ? h[1] : -h[1];
      hullPoints[24 * i + 3 * c + 2] = c & 4 ? h[2] : -h[2];
    }
    hullTris.set(BOX_TRIS, 36 * i);
    hullPointOffset[i + 1] = 8 * (i + 1);
    hullTriOffset[i + 1] = 12 * (i + 1);
  }

  // ---- bonds ----
  const raw = findContacts(boxes, tol);
  const m = raw.length;
  const bondA = new Uint32Array(m), bondB = new Uint32Array(m);
  const bondArea = new Float64Array(m), bondCentroid = new Float64Array(3 * m);
  const bondNormal = new Float64Array(3 * m), bondT1 = new Float64Array(3 * m), bondT2 = new Float64Array(3 * m);
  const bondI1 = new Float64Array(m), bondI2 = new Float64Array(m), bondJ = new Float64Array(m);
  const bondCornerOffset = new Uint32Array(m + 1), bondCorners = new Float64Array(12 * m);
  const bondMaterial = new Uint16Array(m), bondReinforcement = new Uint16Array(m).fill(NO_REINFORCEMENT);

  for (let b = 0; b < m; b++) {
    const { a, b: bb, axis: k, plane, lo, hi } = raw[b];
    const k1 = (k + 1) % 3, k2 = (k + 2) % 3;
    bondA[b] = a; bondB[b] = bb;
    const w1 = hi[0] - lo[0], w2 = hi[1] - lo[1]; // extents along t1, t2
    bondArea[b] = w1 * w2;
    bondCentroid[3 * b + k] = plane;
    bondCentroid[3 * b + k1] = 0.5 * (lo[0] + hi[0]);
    bondCentroid[3 * b + k2] = 0.5 * (lo[1] + hi[1]);
    bondNormal[3 * b + k] = 1; bondT1[3 * b + k1] = 1; bondT2[3 * b + k2] = 1;
    // Rectangle w1 x w2 in (y along t1, z along t2): I1 = ∫z² dA = w1 w2³/12, I2 = ∫y² dA = w2 w1³/12.
    // J = I1 + I2 is the polar moment (overestimates torsion stiffness of thin rectangles; acceptable for chunks).
    bondI1[b] = (w1 * w2 ** 3) / 12; bondI2[b] = (w2 * w1 ** 3) / 12; bondJ[b] = bondI1[b] + bondI2[b];
    const cs: [number, number][] = [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]]; // CCW about n
    cs.forEach(([u, v], c) => {
      bondCorners[12 * b + 3 * c + k] = plane;
      bondCorners[12 * b + 3 * c + k1] = u;
      bondCorners[12 * b + 3 * c + k2] = v;
    });
    bondCornerOffset[b + 1] = 4 * (b + 1);
    const ma = chunkMaterial[a], mb = chunkMaterial[bb];
    const explicit = opts.interfaceFor?.(a, bb) ?? null;
    const mat = explicit ?? interfaceMaterial(table, ma, mb);
    bondMaterial[b] = mat;
    const reinf = table.materials[mat].reinforcement ?? (ma === mb ? table.materials[ma].reinforcement : null);
    if (reinf) bondReinforcement[b] = materialId(table, reinf.material);
  }

  // ---- render faces: split each box face by the overlap rectangles of its bonds ----
  const faceBonds: number[][] = Array.from({ length: n * 6 }, () => []); // face f = 6 i + 2 k + (side: 0 = -k, 1 = +k)
  for (let b = 0; b < m; b++) {
    faceBonds[6 * raw[b].a + 2 * raw[b].axis + 1].push(b);
    faceBonds[6 * raw[b].b + 2 * raw[b].axis].push(b);
  }
  const pos: number[] = [], nrm: number[] = [], tris: number[] = [], triChunk: number[] = [], triTag: number[] = [];
  const renderVertOffset = new Uint32Array(n + 1);
  for (let i = 0; i < n; i++) {
    const { min, max } = boxes[i];
    const cx = chunkPos[3 * i], cy = chunkPos[3 * i + 1], cz = chunkPos[3 * i + 2];
    const centre = [cx, cy, cz];
    let surface = false;
    for (let k = 0; k < 3; k++) {
      const k1 = (k + 1) % 3, k2 = (k + 2) % 3;
      for (let side = 0; side < 2; side++) {
        const fb = faceBonds[6 * i + 2 * k + side];
        const us = uniqueCuts([min[k1], max[k1], ...fb.flatMap((b) => [raw[b].lo[0], raw[b].hi[0]])], tol);
        const vs = uniqueCuts([min[k2], max[k2], ...fb.flatMap((b) => [raw[b].lo[1], raw[b].hi[1]])], tol);
        const w = side ? max[k] : min[k];
        for (let iu = 0; iu + 1 < us.length; iu++) {
          for (let iv = 0; iv + 1 < vs.length; iv++) {
            const u0 = us[iu], u1 = us[iu + 1], v0 = vs[iv], v1 = vs[iv + 1];
            const um = 0.5 * (u0 + u1), vm = 0.5 * (v0 + v1);
            let tag = OUTER_FACE;
            for (const b of fb) {
              const r = raw[b];
              if (um > r.lo[0] && um < r.hi[0] && vm > r.lo[1] && vm < r.hi[1]) { tag = b; break; }
            }
            if (tag === OUTER_FACE) surface = true;
            const base = pos.length / 3;
            const quad: [number, number][] = side ? [[u0, v0], [u1, v0], [u1, v1], [u0, v1]] : [[u0, v0], [u0, v1], [u1, v1], [u1, v0]];
            for (const [u, v] of quad) {
              const p = [0, 0, 0];
              p[k] = w - centre[k]; p[k1] = u - centre[k1]; p[k2] = v - centre[k2];
              pos.push(p[0], p[1], p[2]);
              const nn = [0, 0, 0]; nn[k] = side ? 1 : -1;
              nrm.push(nn[0], nn[1], nn[2]);
            }
            tris.push(base, base + 1, base + 2, base, base + 2, base + 3);
            triChunk.push(i, i); triTag.push(tag, tag);
          }
        }
      }
    }
    if (surface) chunkFlags[i] |= ChunkFlag.SURFACE;
    renderVertOffset[i + 1] = pos.length / 3;
  }

  return {
    chunkCount: n, chunkPos, chunkVolume, chunkMaterial, chunkFlags,
    hullPointOffset, hullPoints, hullTriOffset, hullTris,
    renderVertOffset, renderPos: new Float64Array(pos), renderNormal: new Float64Array(nrm),
    renderTris: new Uint32Array(tris), renderTriChunk: new Uint32Array(triChunk), renderTriTag: new Int32Array(triTag),
    bondCount: m, bondA, bondB, bondArea, bondCentroid, bondNormal, bondT1, bondT2, bondI1, bondI2, bondJ,
    bondCornerOffset, bondCorners, bondMaterial, bondReinforcement,
  };
}
