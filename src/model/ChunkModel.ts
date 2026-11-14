// ChunkModel: the one packed (SoA) format every generator produces (PLAN §1). Headless: no three.js/Rapier.
// Float64 everywhere; conversion to Float32 happens only at the Rapier/GPU boundary.

/** Chunk flag bits stored in `ChunkModel.chunkFlags`. */
export const ChunkFlag = {
  SUPPORT: 1, // anchored to the foundation; eliminated from K in the solve
  SKINNED: 2, // material.deform === 'joints'
  SURFACE: 4, // has at least one outer face (else an interior cell, zero triangles until revealed)
} as const;

/** Face tag of a render triangle that lies on the source surface (outer material). Inner faces store their bondId (>= 0). */
export const OUTER_FACE = -1;

export interface ChunkModel {
  chunkCount: number;
  /** Rest centre x_i: (3n,). */
  chunkPos: Float64Array;
  /** Cell volume (m^3): (n,). */
  chunkVolume: Float64Array;
  /** Index into the MaterialTable: (n,). */
  chunkMaterial: Uint16Array;
  /** ChunkFlag bits: (n,). */
  chunkFlags: Uint8Array;

  /** Collider hull points, chunk-local (relative to chunkPos), CSR by chunk: points of chunk i are [hullPointOffset[i], hullPointOffset[i+1]) (counts of points, xyz each). */
  hullPointOffset: Uint32Array;
  hullPoints: Float64Array;
  /** Hull triangles (local point indices), CSR by chunk in triangles: (n+1,) offsets, 3 indices per triangle. */
  hullTriOffset: Uint32Array;
  hullTris: Uint32Array;

  /** Render mesh, chunk-local vertices (unwelded per face for flat normals), CSR by chunk in vertices. */
  renderVertOffset: Uint32Array;
  renderPos: Float64Array; // (3V,)
  renderNormal: Float64Array; // (3V,)
  /** Render triangles: global vertex indices (3T,), owning chunk (T,), face tag (T,): OUTER_FACE or the bondId of the cut face. */
  renderTris: Uint32Array;
  renderTriChunk: Uint32Array;
  renderTriTag: Int32Array;

  bondCount: number;
  bondA: Uint32Array; // (m,)
  bondB: Uint32Array; // (m,)
  /** Contact face area A (m^2): (m,). */
  bondArea: Float64Array;
  /** Face centroid c: (3m,). */
  bondCentroid: Float64Array;
  /** Face frame R = [n t1 t2], each (3m,); n points from A to B. */
  bondNormal: Float64Array;
  bondT1: Float64Array;
  bondT2: Float64Array;
  /** Second moments of the face about t1 (I1 = ∫z² dA) and t2 (I2 = ∫y² dA), polar J (m^4): (m,). y,z = coords along t1,t2. */
  bondI1: Float64Array;
  bondI2: Float64Array;
  bondJ: Float64Array;
  /** Face polygon corners in world space, CSR: corners of bond b are [bondCornerOffset[b], bondCornerOffset[b+1]) (xyz each). */
  bondCornerOffset: Uint32Array;
  bondCorners: Float64Array;
  /** Bond material id (interface material or derived): (m,). */
  bondMaterial: Uint16Array;
  /** Reinforcement material id, or NO_REINFORCEMENT: (m,). Fraction comes from the base material record. */
  bondReinforcement: Uint16Array;
}

export const NO_REINFORCEMENT = 0xffff;

/** Count of surface (outer-face) chunks; interior cells are the rest. */
export function surfaceChunkCount(model: ChunkModel): number {
  let c = 0;
  for (let i = 0; i < model.chunkCount; i++) if (model.chunkFlags[i] & ChunkFlag.SURFACE) c++;
  return c;
}
