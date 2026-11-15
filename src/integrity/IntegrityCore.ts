// Integrity work behind a message boundary (PLAN §7): the same core runs inline (tests, fallback) or in a worker.
// Main -> core: bonds to remove. Core -> main: bonds broken, detached components, graph version.
// No structure solver yet (an external one will plug in here, reading `graph` and reporting bonds to break).

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import { StructureGraph } from './StructureGraph';

export interface TickRequest {
  type: 'tick';
  /** Bonds to remove (scripted cuts). */
  remove: Uint32Array;
}

export interface TickReply {
  type: 'result';
  /** Graph version: increments on every request that broke bonds. */
  version: number;
  /** Bonds removed by this request. */
  broken: Uint32Array;
  /** Newly detached components: chunks of component k are detachedChunks[detachedOffset[k] .. detachedOffset[k+1]). */
  detachedChunks: Uint32Array;
  detachedOffset: Uint32Array;
}

export interface InitRequest {
  type: 'init';
  model: ChunkModel;
  table: MaterialTable;
}

/** Transferable buffers of a reply (for postMessage). */
export function replyTransfer(r: TickReply): ArrayBuffer[] {
  return [r.broken.buffer, r.detachedChunks.buffer, r.detachedOffset.buffer] as ArrayBuffer[];
}

export class IntegrityCore {
  readonly graph: StructureGraph;
  version = 0;

  /** @param table kept for the future structure solver (materials per chunk/bond). */
  constructor(readonly model: ChunkModel, readonly table: MaterialTable) {
    this.graph = new StructureGraph(model);
  }

  /** Handles one tick: applies removals and runs tier 0. @returns breaks and detached components (copies). */
  tick(req: TickRequest): TickReply {
    const g = this.graph;
    g.removeBonds(req.remove);
    if (g.brokenCount > 0) this.version++;
    return {
      type: 'result', version: this.version,
      broken: g.broken.slice(0, g.brokenCount),
      detachedChunks: g.detachedChunks.slice(0, g.detachedOffset[g.detachedCount]),
      detachedOffset: g.detachedOffset.slice(0, g.detachedCount + 1),
    };
  }
}
