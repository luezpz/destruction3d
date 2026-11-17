// Integrity worker entry (NOTES Design §7): hosts one IntegrityCore; replies transfer their buffers back.
// No `/// <reference lib="webworker" />`: a lib reference applies to the whole tsc program and breaks DOM typing,
// so the worker scope is typed minimally here.

import { InitRequest, IntegrityCore, TickRequest, replyTransfer } from './IntegrityCore';

interface WorkerScope {
  onmessage: ((e: MessageEvent<InitRequest | TickRequest>) => void) | null;
  postMessage(msg: unknown, transfer: Transferable[]): void;
}
const scope = self as unknown as WorkerScope;
let core: IntegrityCore | null = null;

scope.onmessage = (e) => {
  const msg = e.data;
  if (msg.type === 'init') { core = new IntegrityCore(msg.model, msg.table); return; }
  if (!core) throw new Error('integrity worker: tick before init');
  const reply = core.tick(msg);
  scope.postMessage(reply, replyTransfer(reply));
};
