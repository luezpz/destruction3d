// Tier 0 connectivity on the structure graph (NOTES Design §2, §3): removals -> detached components.
import { describe, expect, it } from 'vitest';
import { createMaterialTable, materialId } from '../src/model/Material';
import { Box, generateFromBoxes } from '../src/model/boxGenerator';
import { StructureGraph } from '../src/integrity/StructureGraph';

const table = createMaterialTable();
const concrete = materialId(table, 'concrete');

/** Horizontal chain of N+1 cubes (side h) along x, chunk 0 support. */
function cantilever(N: number, h: number) {
  const boxes: Box[] = [];
  for (let k = 0; k <= N; k++) boxes.push({ min: [k * h, 0, 0], max: [(k + 1) * h, h, h], material: concrete, support: k === 0 });
  return generateFromBoxes(boxes, table);
}

describe('tier 0 connectivity', () => {
  it('removing the wall under a slab drops it; removing a redundant bond does not', () => {
    // two columns (x=0 and x=2) of height 2 under a 3-wide slab at y=2
    const boxes: Box[] = [];
    for (const x of [0, 2]) for (let y = 0; y < 2; y++) boxes.push({ min: [x, y, 0], max: [x + 1, y + 1, 1], material: concrete, support: y === 0 });
    for (let x = 0; x < 3; x++) boxes.push({ min: [x, 2, 0], max: [x + 1, 3, 1], material: concrete, support: false });
    const m = generateFromBoxes(boxes, table);
    const g = new StructureGraph(m);
    const bondBetween = (p: number, q: number) => {
      for (let b = 0; b < m.bondCount; b++) if ((m.bondA[b] === p && m.bondB[b] === q) || (m.bondA[b] === q && m.bondB[b] === p)) return b;
      throw new Error('no bond');
    };
    expect(g.removeBonds([bondBetween(1, 4)])).toBe(0); // slab still held by the right column
    expect(g.removeBonds([bondBetween(3, 6)])).toBe(1); // slab falls, upper column chunks stay
    expect(Array.from(g.detachedChunks.subarray(0, g.detachedOffset[1])).sort()).toEqual([4, 5, 6]);
    for (const c of [4, 5, 6]) expect(g.attached[c]).toBe(0);
    for (const c of [0, 1, 2, 3]) expect(g.attached[c]).toBe(1);
  });

  it('splitting a hanging piece yields separate components', () => {
    const g = new StructureGraph(cantilever(4, 0.1));
    expect(g.removeBonds([0, 2])).toBe(2); // chunks {1,2} and {3,4}
    expect([g.detachedOffset[1] - g.detachedOffset[0], g.detachedOffset[2] - g.detachedOffset[1]]).toEqual([2, 2]);
  });

  it('skips already-broken bonds; breaks inside floating parts are processed and report the split pieces', () => {
    const g = new StructureGraph(cantilever(4, 0.1)); // bonds k: chunk k - chunk k+1
    g.removeBonds([0]); // {1,2,3,4} floats
    expect(g.removeBonds([0, 2])).toBe(0); // bond 0 already gone; bond 2 is inside the floating part: no new detach
    expect(Array.from(g.broken.subarray(0, g.brokenCount))).toEqual([2]);
    expect(g.alive[2]).toBe(0);
    expect(g.floatCount).toBe(2);
    const comp = (k: number) => Array.from(g.floatChunks.subarray(g.floatOffset[k], g.floatOffset[k + 1])).sort();
    expect([comp(0), comp(1)]).toEqual([[1, 2], [3, 4]]);
    // a redundant break inside a floating ring would report one component (no split); here: cut the end of {3,4}
    g.removeBonds([3]);
    expect(g.floatCount).toBe(2); // {3} and {4}
  });
});
