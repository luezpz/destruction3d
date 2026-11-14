// Load assembly helpers (PLAN §2): chunk masses and gravity loads f_i = m_i g.

import { ChunkModel } from '../model/ChunkModel';
import { MaterialTable } from '../model/Material';

/** Chunk masses m_i = density * volume (kg): (n,). */
export function chunkMasses(model: ChunkModel, table: MaterialTable): Float64Array {
  const out = new Float64Array(model.chunkCount);
  for (let i = 0; i < model.chunkCount; i++) out[i] = table.materials[model.chunkMaterial[i]].density * model.chunkVolume[i];
  return out;
}

/** Isotropic own rotational inertia per chunk, cube approximation j = m s^2 / 6 with s = V^(1/3) (kg m^2): (n,). */
export function chunkRotInertia(model: ChunkModel, mass: Float64Array): Float64Array {
  const out = new Float64Array(model.chunkCount);
  for (let i = 0; i < model.chunkCount; i++) out[i] = (mass[i] * Math.cbrt(model.chunkVolume[i]) ** 2) / 6;
  return out;
}

/** Writes gravity loads into f (6n,): force m_i g along -y (g in m/s^2), zero moments. */
export function gravityLoads(mass: Float64Array, g: number, f: Float64Array): void {
  for (let i = 0; i < mass.length; i++) {
    const o = 6 * i;
    f[o] = 0; f[o + 1] = -mass[i] * g; f[o + 2] = 0; f[o + 3] = 0; f[o + 4] = 0; f[o + 5] = 0;
  }
}
