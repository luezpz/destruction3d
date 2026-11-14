// Box-based structure generators (grid wall/slab, running-bond brick wall). All emit Box lists for generateFromBoxes.

import { Box } from './boxGenerator';

export interface GridOptions {
  /** Cell counts along x, y, z. */
  counts: [number, number, number];
  /** Cell size (m) along x, y, z. */
  cell: [number, number, number];
  /** Min corner of the whole block (m). */
  origin?: [number, number, number];
  material: number;
  /** Mark the bottom layer (y index 0) as supports. */
  supportBottom?: boolean;
}

/** Regular grid of equal boxes (walls, slabs, columns). */
export function gridBoxes(o: GridOptions): Box[] {
  const [nx, ny, nz] = o.counts, [sx, sy, sz] = o.cell, [ox, oy, oz] = o.origin ?? [0, 0, 0];
  const out: Box[] = [];
  for (let y = 0; y < ny; y++)
    for (let z = 0; z < nz; z++)
      for (let x = 0; x < nx; x++)
        out.push({
          min: [ox + x * sx, oy + y * sy, oz + z * sz],
          max: [ox + (x + 1) * sx, oy + (y + 1) * sy, oz + (z + 1) * sz],
          material: o.material,
          support: !!o.supportBottom && y === 0,
        });
  return out;
}

export interface BrickWallOptions {
  /** Bricks per course (full-brick count; odd courses get two half bricks at the ends). */
  bricksPerRow: number;
  rows: number;
  /** Brick size (m): length (x), height (y), depth (z). */
  brick: [number, number, number];
  origin?: [number, number, number];
  material: number;
  /** Bottom course is supported. */
  supportBottom?: boolean;
}

/** Running-bond brick wall in the x-y plane: each course offset by half a brick, flush ends via half bricks. */
export function brickWallBoxes(o: BrickWallOptions): Box[] {
  const [L, H, D] = o.brick, [ox, oy, oz] = o.origin ?? [0, 0, 0];
  const width = o.bricksPerRow * L;
  const out: Box[] = [];
  for (let r = 0; r < o.rows; r++) {
    const xs: number[] = [0];
    let x = r % 2 ? 0.5 * L : L;
    while (x < width - 1e-9) { xs.push(x); x += L; }
    xs.push(width);
    for (let i = 0; i + 1 < xs.length; i++)
      out.push({
        min: [ox + xs[i], oy + r * H, oz],
        max: [ox + xs[i + 1], oy + (r + 1) * H, oz + D],
        material: o.material,
        support: !!o.supportBottom && r === 0,
      });
  }
  return out;
}
