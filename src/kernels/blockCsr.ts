// Sparse symmetric matrix of 6x6 blocks in CSR (both triangles stored), indexed by free (non-support) chunks.
// Structure is built once per graph (allocates); assembly and mat-vec are allocation-free kernels.

/** CSR structure of K: (nf, nf) blocks of (6,6). `values` holds 36 doubles per stored block, row-major. */
export interface BlockCsr {
  /** Free block rows. */
  nf: number;
  rowPtr: Uint32Array; // (nf+1,)
  colIdx: Uint32Array; // (nnzBlocks,)
  /** Slot of the diagonal block of each row: (nf,). */
  diagSlot: Uint32Array;
  /** Per bond: slots of blocks [ii, ij, ji, jj]; -1 where a side is a support or the bond is support-support: (4m,). */
  bondSlot: Int32Array;
  values: Float64Array; // (36 nnzBlocks,)
}

/**
 * Builds the block structure from bond adjacency. Supports are eliminated (fixed u = 0).
 * @param freeIdx (n,) free row of each chunk or -1 for supports. @returns structure with zeroed values.
 */
export function buildBlockCsr(nf: number, freeIdx: Int32Array, bondA: Uint32Array, bondB: Uint32Array, bondCount: number): BlockCsr {
  const nbrs: number[][] = Array.from({ length: nf }, (_, i) => [i]);
  for (let b = 0; b < bondCount; b++) {
    const i = freeIdx[bondA[b]], j = freeIdx[bondB[b]];
    if (i >= 0 && j >= 0 && i !== j) { nbrs[i].push(j); nbrs[j].push(i); }
  }
  const rowPtr = new Uint32Array(nf + 1);
  for (let i = 0; i < nf; i++) {
    nbrs[i] = [...new Set(nbrs[i])].sort((x, y) => x - y);
    rowPtr[i + 1] = rowPtr[i] + nbrs[i].length;
  }
  const colIdx = new Uint32Array(rowPtr[nf]), diagSlot = new Uint32Array(nf);
  for (let i = 0; i < nf; i++)
    for (let k = 0; k < nbrs[i].length; k++) {
      colIdx[rowPtr[i] + k] = nbrs[i][k];
      if (nbrs[i][k] === i) diagSlot[i] = rowPtr[i] + k;
    }
  const slot = (i: number, j: number): number => {
    for (let k = rowPtr[i]; k < rowPtr[i + 1]; k++) if (colIdx[k] === j) return k;
    throw new Error(`missing block ${i},${j}`);
  };
  const bondSlot = new Int32Array(4 * bondCount).fill(-1);
  for (let b = 0; b < bondCount; b++) {
    const i = freeIdx[bondA[b]], j = freeIdx[bondB[b]];
    if (i >= 0) bondSlot[4 * b] = diagSlot[i];
    if (j >= 0) bondSlot[4 * b + 3] = diagSlot[j];
    if (i >= 0 && j >= 0) { bondSlot[4 * b + 1] = slot(i, j); bondSlot[4 * b + 2] = slot(j, i); }
  }
  return { nf, rowPtr, colIdx, diagSlot, bondSlot, values: new Float64Array(36 * rowPtr[nf]) };
}

/**
 * K = sum_b scatter(K_b): zeroes `values` then adds bond blocks (bondK: 108 per bond = ii, ij, jj; ji = ij^T).
 * @param bondSlot (4m,) from buildBlockCsr. @param count bonds.
 */
export function assembleBlockCsr(values: Float64Array, bondSlot: Int32Array, bondK: Float64Array, count: number): void {
  values.fill(0);
  for (let b = 0; b < count; b++) {
    const o = 108 * b, sII = bondSlot[4 * b], sIJ = bondSlot[4 * b + 1], sJI = bondSlot[4 * b + 2], sJJ = bondSlot[4 * b + 3];
    if (sII >= 0) for (let k = 0; k < 36; k++) values[36 * sII + k] += bondK[o + k];
    if (sJJ >= 0) for (let k = 0; k < 36; k++) values[36 * sJJ + k] += bondK[o + 72 + k];
    if (sIJ >= 0)
      for (let r = 0; r < 6; r++)
        for (let c = 0; c < 6; c++) {
          const v = bondK[o + 36 + 6 * r + c];
          values[36 * sIJ + 6 * r + c] += v;
          values[36 * sJI + 6 * c + r] += v;
        }
  }
}

/** y = K x for block CSR K; x, y: (6 nf,). */
export function blockMatVec(rowPtr: Uint32Array, colIdx: Uint32Array, values: Float64Array, x: Float64Array, y: Float64Array, nf: number): void {
  for (let i = 0; i < nf; i++) {
    let y0 = 0, y1 = 0, y2 = 0, y3 = 0, y4 = 0, y5 = 0;
    for (let k = rowPtr[i]; k < rowPtr[i + 1]; k++) {
      const v = 36 * k, j = 6 * colIdx[k];
      const x0 = x[j], x1 = x[j + 1], x2 = x[j + 2], x3 = x[j + 3], x4 = x[j + 4], x5 = x[j + 5];
      y0 += values[v] * x0 + values[v + 1] * x1 + values[v + 2] * x2 + values[v + 3] * x3 + values[v + 4] * x4 + values[v + 5] * x5;
      y1 += values[v + 6] * x0 + values[v + 7] * x1 + values[v + 8] * x2 + values[v + 9] * x3 + values[v + 10] * x4 + values[v + 11] * x5;
      y2 += values[v + 12] * x0 + values[v + 13] * x1 + values[v + 14] * x2 + values[v + 15] * x3 + values[v + 16] * x4 + values[v + 17] * x5;
      y3 += values[v + 18] * x0 + values[v + 19] * x1 + values[v + 20] * x2 + values[v + 21] * x3 + values[v + 22] * x4 + values[v + 23] * x5;
      y4 += values[v + 24] * x0 + values[v + 25] * x1 + values[v + 26] * x2 + values[v + 27] * x3 + values[v + 28] * x4 + values[v + 29] * x5;
      y5 += values[v + 30] * x0 + values[v + 31] * x1 + values[v + 32] * x2 + values[v + 33] * x3 + values[v + 34] * x4 + values[v + 35] * x5;
    }
    const o = 6 * i;
    y[o] = y0; y[o + 1] = y1; y[o + 2] = y2; y[o + 3] = y3; y[o + 4] = y4; y[o + 5] = y5;
  }
}
