// Demo structures for the debug app.

import type { ChunkModel } from './model/ChunkModel';
import { MaterialTable, materialId } from './model/Material';
import { generateFromBoxes } from './model/boxGenerator';
import { brickWallBoxes, gridBoxes } from './model/generators';

export const SCENES = ['concrete wall', 'brick wall', 'glass pane', 'concrete block 6^3'] as const;
export type SceneName = (typeof SCENES)[number];

/** Builds the named demo structure, centred on x/z, standing on y = 0. */
export function buildScene(name: SceneName, table: MaterialTable): ChunkModel {
  const id = (n: string) => materialId(table, n);
  switch (name) {
    case 'concrete wall':
      return generateFromBoxes(gridBoxes({ counts: [10, 8, 2], cell: [0.5, 0.5, 0.15], origin: [-2.5, 0, -0.15], material: id('concrete'), supportBottom: true }), table);
    case 'brick wall': {
      const mortar = id('mortar');
      return generateFromBoxes(brickWallBoxes({ bricksPerRow: 12, rows: 30, brick: [0.22, 0.075, 0.105], origin: [-1.32, 0, -0.0525], material: id('brick'), supportBottom: true }), table, { interfaceFor: () => mortar });
    }
    case 'glass pane':
      return generateFromBoxes(gridBoxes({ counts: [12, 10, 1], cell: [0.15, 0.15, 0.01], origin: [-0.9, 0, -0.005], material: id('glass'), supportBottom: true }), table);
    case 'concrete block 6^3':
      return generateFromBoxes(gridBoxes({ counts: [6, 6, 6], cell: [0.4, 0.4, 0.4], origin: [-1.2, 0, -1.2], material: id('concrete'), supportBottom: true }), table);
  }
}
