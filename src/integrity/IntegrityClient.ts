// Main-thread side of the integrity boundary: one request in flight, replies polled each physics step.
// `InlineIntegrity` runs the core synchronously (tests, no-worker fallback); `WorkerIntegrity` posts to a worker.

import type { ChunkModel } from '../model/ChunkModel';
import type { MaterialTable } from '../model/Material';
import { InitRequest, IntegrityCore, TickReply, TickRequest } from './IntegrityCore';

export interface IntegrityClient {
  /** Sends a tick request; the removal buffer is transferred. */
  request(req: TickRequest): void;
  /** Returns the reply if one arrived since the last poll. */
  poll(): TickReply | null;
  dispose(): void;
  /** True when request() computes the reply immediately (poll right after request returns it). */
  readonly sync: boolean;
}

/** Synchronous client: the reply is ready on the next poll. */
export class InlineIntegrity implements IntegrityClient {
  readonly core: IntegrityCore;
  readonly sync = true;
  private reply: TickReply | null = null;
  constructor(model: ChunkModel, table: MaterialTable) { this.core = new IntegrityCore(model, table); }
  request(req: TickRequest): void { this.reply = this.core.tick(req); }
  poll(): TickReply | null { const r = this.reply; this.reply = null; return r; }
  dispose(): void {}
}

/** Worker client: model + table are structured-cloned once at init; tick buffers are transferred both ways. */
export class WorkerIntegrity implements IntegrityClient {
  private readonly worker: Worker;
  readonly sync = false;
  private reply: TickReply | null = null;
  constructor(model: ChunkModel, table: MaterialTable) {
    this.worker = new Worker(new URL('./integrity.worker.ts', import.meta.url), { type: 'module' });
    this.worker.onmessage = (e: MessageEvent<TickReply>) => { this.reply = e.data; };
    this.worker.onerror = (e) => console.error('integrity worker', e);
    const init: InitRequest = { type: 'init', model, table };
    this.worker.postMessage(init);
  }
  request(req: TickRequest): void { this.worker.postMessage(req, [req.remove.buffer as ArrayBuffer]); }
  poll(): TickReply | null { const r = this.reply; this.reply = null; return r; }
  dispose(): void { this.worker.terminate(); }
}
