// Damage kernel (PLAN §2): low-passed ratio -> damage growth -> bond removal, plus break propagation.
// Pure typed-array functions, no allocation per call.
//
// rf += (r - rf) * min(1, dt / filterTime)          (first-order low-pass, ~filterTime window)
// d  += damageRate * max(0, rf - 1) * dt            (damageRate = Infinity: d = 1 as soon as rf > 1)
// d >= 1 -> bond removed (alive = 0), appended to `broken`; neighbours (bonds sharing a chunk) get d += propagation.

import { MaterialViews } from './failureRatio';

/** Chunk -> incident bonds adjacency, CSR: bonds of chunk i are adjBond[adjOffset[i] .. adjOffset[i+1]). */
export interface BondAdjacency {
  adjOffset: Uint32Array; // (n+1,)
  adjBond: Uint32Array; // (2m,)
}

/** Builds the chunk -> bond adjacency (setup-time, allocates). */
export function buildBondAdjacency(n: number, bondA: Uint32Array, bondB: Uint32Array, m: number): BondAdjacency {
  const adjOffset = new Uint32Array(n + 1), adjBond = new Uint32Array(2 * m), fill = new Uint32Array(n);
  for (let b = 0; b < m; b++) { adjOffset[bondA[b] + 1]++; adjOffset[bondB[b] + 1]++; }
  for (let i = 0; i < n; i++) adjOffset[i + 1] += adjOffset[i];
  for (let b = 0; b < m; b++) {
    adjBond[adjOffset[bondA[b]] + fill[bondA[b]]++] = b;
    adjBond[adjOffset[bondB[b]] + fill[bondB[b]]++] = b;
  }
  return { adjOffset, adjBond };
}

/**
 * Advances filtered ratios `rf` (m,) and damage `d` (m,) by dt (s) from raw ratios `r` (m,).
 * Newly broken bonds get alive = 0 and are written to `broken`; propagation damage is added to their live neighbours
 * (which may break them in the same call, chain shatter). @returns number of broken bonds written (capped at broken.length).
 */
export function updateDamage(mat: MaterialViews, bondMaterial: Uint16Array, bondA: Uint32Array, bondB: Uint32Array,
  adj: BondAdjacency, r: Float64Array, rf: Float64Array, d: Float64Array, alive: Uint8Array, dt: number,
  broken: Uint32Array, count: number): number {
  let nb = 0;
  for (let b = 0; b < count; b++) {
    if (!alive[b]) continue;
    const id = bondMaterial[b];
    rf[b] += (r[b] - rf[b]) * Math.min(1, dt / mat.filterTime[id]);
    const over = rf[b] - 1;
    if (over > 0) d[b] = mat.damageRate[id] === Infinity ? 1 : d[b] + mat.damageRate[id] * over * dt;
    if (d[b] >= 1 && nb < broken.length) { alive[b] = 0; broken[nb++] = b; }
  }
  // Propagation: breadth-first over the growing broken list so chains resolve in one call.
  for (let q = 0; q < nb; q++) {
    const b = broken[q], p = mat.propagation[bondMaterial[b]];
    if (p <= 0) continue;
    for (let side = 0; side < 2; side++) {
      const c = side ? bondB[b] : bondA[b];
      for (let k = adj.adjOffset[c]; k < adj.adjOffset[c + 1]; k++) {
        const o = adj.adjBond[k];
        if (!alive[o]) continue;
        d[o] += p;
        if (d[o] >= 1 && nb < broken.length) { alive[o] = 0; broken[nb++] = o; }
      }
    }
  }
  return nb;
}
