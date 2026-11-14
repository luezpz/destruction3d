// Culled two-material index buffers (PLAN §6). Headless (no three.js) so it is unit-testable.
// Outer index buffer: static, built once. Inner buffer: append-only; a bond's cut faces (both sides) are copied in on reveal.

import { ChunkModel, OUTER_FACE } from '../model/ChunkModel';

export interface CullBuffers {
  /** Outer-material triangles (vertex indices), static. */
  outerIndex: Uint32Array;
  /** All inner triangles sorted by bond: the source for reveals. Bond b owns [bondRange[b], bondRange[b+1]) (index units). */
  innerSorted: Uint32Array;
  bondRange: Uint32Array;
  /** Inner draw buffer (capacity = innerSorted.length); [0, innerCount) is drawn. Monotonic. */
  innerIndex: Uint32Array;
  innerCount: number;
  /** 1 once a bond's faces are appended (idempotent reveal). */
  revealed: Uint8Array;
}

/** Splits the model's render triangles into the static outer buffer and the bond-sorted inner source. */
export function buildCullBuffers(model: ChunkModel): CullBuffers {
  const T = model.renderTriTag.length, m = model.bondCount;
  let outerTris = 0;
  const bondRange = new Uint32Array(m + 1);
  for (let t = 0; t < T; t++) {
    const tag = model.renderTriTag[t];
    if (tag === OUTER_FACE) outerTris++;
    else bondRange[tag + 1] += 3;
  }
  for (let b = 0; b < m; b++) bondRange[b + 1] += bondRange[b];
  const outerIndex = new Uint32Array(3 * outerTris);
  const innerSorted = new Uint32Array(bondRange[m]);
  const cursor = bondRange.slice(0, m);
  let o = 0;
  for (let t = 0; t < T; t++) {
    const tag = model.renderTriTag[t];
    const dst = tag === OUTER_FACE ? outerIndex : innerSorted;
    const at = tag === OUTER_FACE ? (o += 3) - 3 : (cursor[tag] += 3) - 3;
    dst[at] = model.renderTris[3 * t]; dst[at + 1] = model.renderTris[3 * t + 1]; dst[at + 2] = model.renderTris[3 * t + 2];
  }
  return { outerIndex, innerSorted, bondRange, innerIndex: new Uint32Array(innerSorted.length), innerCount: 0, revealed: new Uint8Array(m) };
}

/**
 * Reveals bond b's cut faces (call when the bond breaks or its damage passes d_show).
 * @returns the appended index range [start, start+count) to upload, or null if already revealed.
 */
export function revealBond(c: CullBuffers, b: number): { start: number; count: number } | null {
  if (c.revealed[b]) return null;
  c.revealed[b] = 1;
  const s = c.bondRange[b], e = c.bondRange[b + 1];
  const start = c.innerCount;
  c.innerIndex.set(c.innerSorted.subarray(s, e), start);
  c.innerCount += e - s;
  return { start, count: e - s };
}
