// Demo structures for the debug app. Each scene is a box list (mixed materials allowed; bonds come from face overlap)
// plus a camera framing. Supports = boxes touching the ground (y = 0).

import type { ChunkModel } from './model/ChunkModel';
import { MaterialTable, materialId } from './model/Material';
import { Box, generateFromBoxes } from './model/boxGenerator';
import { brickWallBoxes, gridBoxes } from './model/generators';

export const SCENES = [
  'concrete wall', 'brick wall', 'glass pane', 'concrete block 6^3',
  'slab on columns', 'tower', 'bridge', 'cantilever balcony', 'house', 'brick gate', 'ice pillars', 'steel plate',
  'steel cantilever', 'steel beam', 'steel truss bridge', 'steel frame', 'steel tower', 'steel pole', 'rubber curtain',
] as const;
export type SceneName = (typeof SCENES)[number];

type V3 = [number, number, number];

/**
 * Per-scene strength multipliers (material name -> factor) that the debug app loads into its "strength per material" sliders.
 * Real steel never yields from a ball (NOTES Gotchas), so the joint demos weaken it until the hit part bends plastically
 * but still holds its own weight (tests/jointScenes.test.ts checks both).
 */
export const SCENE_STRENGTH: Partial<Record<SceneName, Record<string, number>>> = {
  'steel plate': { steel: 0.01 },
  'steel cantilever': { steel: 0.05 },
  'steel beam': { steel: 0.03 },
  'steel truss bridge': { steel: 0.02 },
  'steel frame': { steel: 0.02 },
  'steel tower': { steel: 0.01 },
  'steel pole': { steel: 0.08 },
};

export interface DemoScene {
  model: ChunkModel;
  /** Camera position and orbit target (m). */
  camera: V3;
  target: V3;
}

/** Grid block of `counts` cells of size `cell` starting at min corner `min`; bottom layer supported when it touches y = 0. */
function block(min: V3, counts: V3, cell: V3, material: number): Box[] {
  return gridBoxes({ counts, cell, origin: min, material, supportBottom: min[1] <= 1e-9 });
}

/** Removes boxes whose centre lies inside the axis-aligned region [lo, hi] (doors, windows). */
function carve(boxes: Box[], lo: V3, hi: V3): Box[] {
  return boxes.filter((b) => {
    for (let a = 0; a < 3; a++) {
      const c = 0.5 * (b.min[a] + b.max[a]);
      if (c < lo[a] || c > hi[a]) return true;
    }
    return false;
  });
}

/** Builds the named demo structure (centred on x/z, standing on y = 0) and its camera framing. */
export function buildScene(name: SceneName, table: MaterialTable): DemoScene {
  const id = (n: string) => materialId(table, n);
  const gen = (boxes: Box[], camera: V3, target: V3, interfaceFor?: () => number): DemoScene =>
    ({ model: generateFromBoxes(boxes, table, interfaceFor ? { interfaceFor } : {}), camera, target });
  switch (name) {
    case 'concrete wall':
      return gen(block([-2.5, 0, -0.15], [10, 8, 2], [0.5, 0.5, 0.15], id('concrete')), [5, 3.5, 7], [0, 1.5, 0]);
    case 'brick wall': {
      const mortar = id('mortar');
      const boxes = brickWallBoxes({ bricksPerRow: 12, rows: 30, brick: [0.22, 0.075, 0.105], origin: [-1.32, 0, -0.0525], material: id('brick'), supportBottom: true });
      return gen(boxes, [2.5, 1.8, 3.5], [0, 1, 0], () => mortar);
    }
    case 'steel plate': // deform 'joints': skinned (multi-influence) vertices; rigid until joint chains land (M4)
      return gen(block([-1, 0, -0.015], [10, 10, 1], [0.2, 0.2, 0.03], id('steel')), [2.2, 1.5, 3], [0, 1, 0]);
    case 'glass pane':
      return gen(block([-0.9, 0, -0.005], [12, 10, 1], [0.15, 0.15, 0.01], id('glass')), [1.8, 1.2, 2.4], [0, 0.75, 0]);
    case 'concrete block 6^3':
      return gen(block([-1.2, 0, -1.2], [6, 6, 6], [0.4, 0.4, 0.4], id('concrete')), [5, 3.5, 7], [0, 1.2, 0]);
    case 'slab on columns': {
      // Reinforced slab 4 x 4 m on four 0.4 m columns; shift+click a column chunk to see the corner drop.
      const rc = id('reinforcedConcrete'), c = id('concrete');
      const boxes: Box[] = [];
      for (const x of [-2, 1.6]) for (const z of [-2, 1.6]) boxes.push(...block([x, 0, z], [1, 6, 1], [0.4, 0.4, 0.4], c));
      boxes.push(...block([-2, 2.4, -2], [10, 1, 10], [0.4, 0.25, 0.4], rc));
      return gen(boxes, [6, 4.5, 7], [0, 1.5, 0]);
    }
    case 'tower':
      // Slender concrete tower 0.8 x 8 x 0.8 m: a hit low down cracks the base and topples it.
      return gen(block([-0.4, 0, -0.4], [2, 20, 2], [0.4, 0.4, 0.4], id('concrete')), [7, 4.5, 9], [0, 3.5, 0]);
    case 'bridge': {
      // Two brick piers carrying a concrete deck with overhangs at both ends.
      const c = id('concrete'), brick = id('brick');
      const boxes = [
        ...block([-2.4, 0, -0.6], [2, 6, 3], [0.4, 0.4, 0.4], brick),
        ...block([1.6, 0, -0.6], [2, 6, 3], [0.4, 0.4, 0.4], brick),
        ...block([-4, 2.4, -0.8], [20, 1, 4], [0.4, 0.3, 0.4], c),
      ];
      return gen(boxes, [6, 5, 9], [0, 1.8, 0]);
    }
    case 'cantilever balcony': {
      // Wall with a long thin plain-concrete balcony: hit the tip and it cracks at the root and falls.
      const c = id('concrete');
      const boxes = [
        ...block([-1.5, 0, -0.6], [6, 10, 2], [0.5, 0.4, 0.3], c),
        ...block([-1.0, 2.8, 0], [4, 1, 7], [0.5, 0.2, 0.4], c),
      ];
      return gen(boxes, [6, 4, 6], [0, 2.4, 1]);
    }
    case 'house': {
      // Brick-sized concrete blocks: four walls with door and windows, slab roof.
      const c = id('concrete'), rc = id('reinforcedConcrete');
      const cell: V3 = [0.4, 0.4, 0.2], cellZ: V3 = [0.2, 0.4, 0.4];
      let walls: Box[] = [
        ...block([-2, 0, -2], [10, 7, 1], cell, c),           // back (z = -2)
        ...block([-2, 0, 1.8], [10, 7, 1], cell, c),          // front (z = 1.8)
        ...block([-2, 0, -1.8], [1, 7, 9], cellZ, c),         // left
        ...block([1.8, 0, -1.8], [1, 7, 9], cellZ, c),        // right
      ];
      walls = carve(walls, [-0.4, 0, 1.7], [0.4, 2.0, 2.1]);   // door
      walls = carve(walls, [-1.6, 1.2, 1.7], [-0.8, 2.0, 2.1]); // front windows
      walls = carve(walls, [0.8, 1.2, 1.7], [1.6, 2.0, 2.1]);
      walls = carve(walls, [1.7, 1.2, -0.8], [2.1, 2.0, 0.8]);  // side window
      walls.push(...block([-2, 2.8, -2], [10, 1, 10], [0.4, 0.2, 0.4], rc));
      return gen(walls, [6, 4.5, 8], [0, 1.4, 0]);
    }
    case 'brick gate': {
      // Two brick piers with a brick lintel course; mortar joints are the weak links.
      const brick = id('brick'), mortar = id('mortar');
      const pier = (x: number) => brickWallBoxes({ bricksPerRow: 3, rows: 24, brick: [0.22, 0.075, 0.22], origin: [x, 0, -0.11], material: brick, supportBottom: true });
      const lintel = brickWallBoxes({ bricksPerRow: 11, rows: 4, brick: [0.22, 0.075, 0.22], origin: [-1.21, 1.8, -0.11], material: brick });
      return gen([...pier(-1.21), ...pier(0.55), ...lintel], [2.5, 1.8, 3.5], [0, 1.1, 0], () => mortar);
    }
    case 'steel cantilever': {
      // Steel beam 3.6 x 0.45 m welded to a concrete wall: a hit on the tip bends it plastically at the root.
      const c = id('concrete'), steel = id('steel');
      const boxes = [
        ...block([-1.6, 0, -0.4], [3, 8, 2], [0.4, 0.4, 0.4], c),
        ...block([-0.4, 2.0, -0.15], [24, 3, 1], [0.15, 0.15, 0.3], steel),
      ];
      return gen(boxes, [1.5, 3, 8], [0.8, 2.2, 0]);
    }
    case 'steel beam': {
      // Steel girder 6 x 0.4 m resting on two concrete piers (fixed at both ends in the model): a hit mid-span sags it.
      const c = id('concrete'), steel = id('steel');
      const boxes = [
        ...block([-3, 0, -0.4], [2, 5, 2], [0.4, 0.4, 0.4], c),
        ...block([2.2, 0, -0.4], [2, 5, 2], [0.4, 0.4, 0.4], c),
        ...block([-3, 2.0, -0.2], [30, 2, 1], [0.2, 0.2, 0.4], steel),
      ];
      return gen(boxes, [1.5, 3.2, 11], [0, 2.2, 0]);
    }
    case 'steel truss bridge': {
      // Vierendeel truss (axis-aligned boxes only, no diagonals): two chord/post trusses, cross beams on top, plate deck, concrete abutments.
      const c = id('concrete'), steel = id('steel');
      const s = 0.4, posts = [0, 4, 8, 12, 15];
      const boxes: Box[] = [
        ...block([-4.0, 0, -1.2], [3, 3, 6], [s, s, s], c),
        ...block([2.8, 0, -1.2], [3, 3, 6], [s, s, s], c),
        ...block([-3.2, 1.2, -0.8], [8, 1, 4], [0.8, 0.1, 0.4], steel), // deck
      ];
      for (const z of [-1.0, 0.8]) {
        boxes.push(...block([-3.2, 1.2, z], [16, 1, 1], [s, s, 0.2], steel), ...block([-3.2, 2.0, z], [16, 1, 1], [s, s, 0.2], steel)); // chords
        for (const i of posts) boxes.push(...block([-3.2 + i * s, 1.6, z], [1, 1, 1], [s, s, 0.2], steel));
      }
      for (const i of posts) boxes.push(...block([-3.2 + i * s, 2.0, -0.8], [1, 1, 4], [s, s, s], steel)); // top cross beams
      return gen(boxes, [6, 3.5, 7.5], [0, 1.5, 0]);
    }
    case 'steel frame': {
      // Moment frame 2.4 x 2.4 x 3 m: four corner columns, a floor ring and a roof ring of 0.3 m steel cubes.
      const steel = id('steel'), s = 0.3, o = -1.2, e = o + 7 * s;
      const ring = (level: number): Box[] => [
        ...block([o, level * s, o], [8, 1, 1], [s, s, s], steel), ...block([o, level * s, e], [8, 1, 1], [s, s, s], steel),
        ...block([o, level * s, o + s], [1, 1, 6], [s, s, s], steel), ...block([e, level * s, o + s], [1, 1, 6], [s, s, s], steel),
      ];
      const boxes = [...ring(4), ...ring(9)];
      for (const x of [o, e]) for (const z of [o, e]) boxes.push(...block([x, 0, z], [1, 4, 1], [s, s, s], steel), ...block([x, 5 * s, z], [1, 4, 1], [s, s, s], steel));
      return gen(boxes, [4.5, 2.8, 5.5], [0, 1.3, 0]);
    }
    case 'steel tower': {
      // Slender steel lattice tower (4 legs, 3 rings) 2.4 x 2.4 x 6.8 m: knock a leg and it bends or topples.
      const steel = id('steel'), s = 0.4, o = -1.2, e = o + 5 * s;
      const ring = (level: number): Box[] => [
        ...block([o, level * s, o], [6, 1, 1], [s, s, s], steel), ...block([o, level * s, e], [6, 1, 1], [s, s, s], steel),
        ...block([o, level * s, o + s], [1, 1, 4], [s, s, s], steel), ...block([e, level * s, o + s], [1, 1, 4], [s, s, s], steel),
      ];
      const boxes = [...ring(5), ...ring(11), ...ring(16)];
      for (const x of [o, e]) for (const z of [o, e])
        boxes.push(...block([x, 0, z], [1, 5, 1], [s, s, s], steel), ...block([x, 6 * s, z], [1, 5, 1], [s, s, s], steel), ...block([x, 12 * s, z], [1, 4, 1], [s, s, s], steel));
      return gen(boxes, [8, 4, 9], [0, 3, 0]);
    }
    case 'steel pole': {
      // Street-lamp pole (0.3 m square, 3.6 m) with an arm on a concrete footing: a hit bends it permanently.
      const c = id('concrete'), steel = id('steel'), s = 0.3;
      const boxes = [
        ...block([-0.8, 0, -0.8], [4, 1, 4], [0.4, 0.4, 0.4], c),
        ...block([-0.15, 0.4, -0.15], [1, 12, 1], [s, s, s], steel),
        ...block([0.15, 3.7, -0.15], [5, 1, 1], [s, s, s], steel),
      ];
      return gen(boxes, [4.5, 2.4, 6], [0, 2, 0]);
    }
    case 'rubber curtain': {
      // Industrial strip curtain: nine 2 m rubber strips hang from a concrete lintel. The joint chains are inextensible
      // (translation locked), so a taut sheet can't stretch; free-hanging strips swing and flex instead.
      const c = id('concrete'), rubber = id('rubber');
      const boxes = [
        ...block([-2.2, 0, -0.4], [1, 6, 1], [0.4, 0.4, 0.4], c), ...block([1.4, 0, -0.4], [1, 6, 1], [0.4, 0.4, 0.4], c),
        ...block([-2.2, 2.4, -0.4], [10, 1, 1], [0.4, 0.4, 0.4], c),
      ];
      for (let i = 0; i < 9; i++) boxes.push(...block([-1.7 + i * 0.4, 0.4, -0.05], [1, 10, 1], [0.3, 0.2, 0.1], rubber));
      return gen(boxes, [2, 1.8, 6.5], [0, 1.4, 0]);
    }
    case 'ice pillars': {
      // Ice columns under a wood beam: brittle ice shatters, the beam drops.
      const ice = id('ice'), wood = id('wood');
      const boxes: Box[] = [];
      for (const x of [-1.5, -0.3, 0.9]) boxes.push(...block([x, 0, -0.3], [2, 8, 2], [0.3, 0.25, 0.3], ice));
      boxes.push(...block([-1.8, 2, -0.3], [12, 1, 2], [0.3, 0.2, 0.3], wood));
      return gen(boxes, [4, 2.5, 5], [0, 1.2, 0]);
    }
  }
}
