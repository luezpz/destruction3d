// Integrity work behind a message boundary (PLAN §7): the same core runs inline (tests, fallback) or in a worker.
// Main -> core: bonds to remove, impacts. Core -> main: bonds broken, detached components, dent poses, graph version.
// No structure solver yet (an external one will plug in here, reading `graph` and reporting bonds to break).

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import { StructureGraph } from './StructureGraph';
import { DentField } from './DentField';

export interface TickRequest {
  type: 'tick';
  /** Bonds to remove (scripted cuts). */
  remove: Uint32Array;
  /** Impacts on static chunks, IMPACT_STRIDE doubles each: chunk, point (3), unit push dir (3), impulse N s, force N. */
  impacts?: Float64Array;
}

export const IMPACT_STRIDE = 9;

export interface TickReply {
  type: 'result';
  /** Graph version: increments on every request that broke bonds. */
  version: number;
  /** Bonds removed by this request. */
  broken: Uint32Array;
  /** Newly detached components: chunks of component k are detachedChunks[detachedOffset[k] .. detachedOffset[k+1]). */
  detachedChunks: Uint32Array;
  detachedOffset: Uint32Array;
  /** Components of already-floating parts touched by this request's breaks (same layout); 2+ per old part = it split. */
  floatChunks: Uint32Array;
  floatOffset: Uint32Array;
  /** Chunks whose fake dent pose changed (PLAN §4) and their offset o (3) + tilt th (3) per chunk, structure frame. */
  poseChunks: Uint32Array;
  poses: Float64Array;
}

export interface InitRequest {
  type: 'init';
  model: ChunkModel;
  table: MaterialTable;
}

/** Transferable buffers of a reply (for postMessage). */
export function replyTransfer(r: TickReply): ArrayBuffer[] {
  return [r.broken.buffer, r.detachedChunks.buffer, r.detachedOffset.buffer, r.floatChunks.buffer, r.floatOffset.buffer, r.poseChunks.buffer, r.poses.buffer] as ArrayBuffer[];
}

export class IntegrityCore {
  readonly graph: StructureGraph;
  readonly dents: DentField;
  version = 0;

  /** @param table kept for the future structure solver (materials per chunk/bond). */
  constructor(readonly model: ChunkModel, readonly table: MaterialTable) {
    this.graph = new StructureGraph(model);
    this.dents = new DentField(model, table, this.graph.adj);
  }

  /** Handles one tick: dents from impacts, then removals + dent cracks, tier 0. @returns breaks, components, dent poses (copies). */
  tick(req: TickRequest): TickReply {
    const g = this.graph, d = this.dents, im = req.impacts;
    if (im) for (let o = 0; o + IMPACT_STRIDE <= im.length; o += IMPACT_STRIDE) d.apply(im[o], im, o + 1, im, o + 4, im[o + 7], im[o + 8], g.attached, g.alive);
    let remove = req.remove;
    if (d.breakCount) {
      remove = new Uint32Array(req.remove.length + d.breakCount);
      remove.set(req.remove); remove.set(d.breaks.subarray(0, d.breakCount), req.remove.length);
      d.clearBreaks();
    }
    g.removeBonds(remove);
    const poseChunks = d.dirty.slice(0, d.dirtyCount), poses = new Float64Array(6 * poseChunks.length);
    poseChunks.forEach((c, k) => { poses.set(d.offset.subarray(3 * c, 3 * c + 3), 6 * k); poses.set(d.tilt.subarray(3 * c, 3 * c + 3), 6 * k + 3); });
    d.clearDirty();
    if (g.brokenCount > 0) this.version++;
    return {
      type: 'result', version: this.version,
      broken: g.broken.slice(0, g.brokenCount),
      detachedChunks: g.detachedChunks.slice(0, g.detachedOffset[g.detachedCount]),
      detachedOffset: g.detachedOffset.slice(0, g.detachedCount + 1),
      floatChunks: g.floatChunks.slice(0, g.floatOffset[g.floatCount]),
      floatOffset: g.floatOffset.slice(0, g.floatCount + 1),
      poseChunks, poses,
    };
  }
}
