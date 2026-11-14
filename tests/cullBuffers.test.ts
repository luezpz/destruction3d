import { describe, expect, it } from 'vitest';
import { OUTER_FACE } from '../src/model/ChunkModel';
import { createMaterialTable, materialId } from '../src/model/Material';
import { generateFromBoxes } from '../src/model/boxGenerator';
import { gridBoxes } from '../src/model/generators';
import { buildCullBuffers, revealBond } from '../src/render/cullBuffers';

/** Deterministic PRNG (mulberry32) for reproducible random break sequences. */
function rng(seed: number): () => number {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Canonical multiset of triangles (sorted index triples) for set comparison. */
function triSet(idx: ArrayLike<number>, count: number): string[] {
  const out: string[] = [];
  for (let i = 0; i < count; i += 3) out.push([idx[i], idx[i + 1], idx[i + 2]].join(','));
  return out.sort();
}

describe('culled two-material buffers', () => {
  const t = createMaterialTable();
  const model = generateFromBoxes(gridBoxes({ counts: [4, 4, 4], cell: [1, 1, 1], material: materialId(t, 'concrete') }), t);

  it('initial draw = outer triangles only; interior cells contribute zero', () => {
    const c = buildCullBuffers(model);
    expect(c.innerCount).toBe(0);
    expect(c.outerIndex.length / 3).toBe(6 * 16 * 2); // 6 sides x 16 cells x 2 tris
    const used = new Set<number>();
    for (const v of c.outerIndex) used.add(v);
    for (let i = 0; i < model.chunkCount; i++) {
      const interior = [i % 4, Math.floor(i / 16), Math.floor(i / 4) % 4].every((q) => q > 0 && q < 3);
      if (!interior) continue;
      for (let v = model.renderVertOffset[i]; v < model.renderVertOffset[i + 1]; v++) expect(used.has(v)).toBe(false);
    }
  });

  it('reveal appends exactly the bond faces (both sides), idempotent, monotonic', () => {
    const c = buildCullBuffers(model);
    const r = revealBond(c, 5)!;
    expect(r).toEqual({ start: 0, count: 12 }); // 2 sides x 2 tris x 3
    const owners = new Set<number>();
    for (let i = 0; i < 12; i += 3) {
      const tri = [...Array(model.renderTriTag.length).keys()].find((tt) => model.renderTris[3 * tt] === c.innerIndex[i])!;
      expect(model.renderTriTag[tri]).toBe(5);
      owners.add(model.renderTriChunk[tri]);
    }
    expect(owners).toEqual(new Set([model.bondA[5], model.bondB[5]]));
    expect(revealBond(c, 5)).toBeNull();
    expect(c.innerCount).toBe(12);
  });

  it('random break sequences match a brute-force recompute', () => {
    const rand = rng(42);
    for (let trial = 0; trial < 5; trial++) {
      const c = buildCullBuffers(model);
      const broken = new Set<number>();
      let prev = 0;
      for (let k = 0; k < 60; k++) {
        const b = Math.floor(rand() * model.bondCount);
        broken.add(b);
        revealBond(c, b);
        expect(c.innerCount).toBeGreaterThanOrEqual(prev);
        prev = c.innerCount;
      }
      const brute: number[] = [];
      for (let tt = 0; tt < model.renderTriTag.length; tt++) {
        const tag = model.renderTriTag[tt];
        if (tag !== OUTER_FACE && broken.has(tag)) brute.push(model.renderTris[3 * tt], model.renderTris[3 * tt + 1], model.renderTris[3 * tt + 2]);
      }
      expect(triSet(c.innerIndex, c.innerCount)).toEqual(triSet(brute, brute.length));
    }
  });
});
