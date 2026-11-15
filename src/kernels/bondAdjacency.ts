// Chunk -> incident bonds adjacency (CSR), shared by connectivity and any graph walk over bonds.

/** Chunk -> incident bonds adjacency, CSR: bonds of chunk i are adjBond[adjOffset[i] .. adjOffset[i+1]). */
export interface BondAdjacency {
  adjOffset: Uint32Array; // (n+1,)
  adjBond: Uint32Array; // (2m,)
}

/** Builds the adjacency (setup-time, allocates). @param n chunks; @param bondA/bondB (m,) bond ends; @param m bonds. */
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
