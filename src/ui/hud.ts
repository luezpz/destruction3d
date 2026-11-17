// On-screen overlay of the debug app: scene title, frame timing and key destruction counters (top left), input hints (bottom).
import './style.css';

/** Live numbers shown in the HUD tiles (accent = highlighted tile). */
export interface HudValues {
  fps: number;
  simMs: string;
  brokenBonds: number;
  islands: number;
  joints: number;
  collisionBreaks: number;
}

const TILES: { key: keyof HudValues; label: string; accent?: boolean }[] = [
  { key: 'fps', label: 'fps' }, { key: 'simMs', label: 'sim ms / frame' },
  { key: 'brokenBonds', label: 'bonds broken', accent: true }, { key: 'islands', label: 'islands' },
  { key: 'joints', label: 'joints alive' }, { key: 'collisionBreaks', label: 'impact cuts' },
];

export interface Hud {
  /** Sets the scene name and paused badge. */
  setScene(name: string): void;
  setPaused(p: boolean): void;
  /** Writes the tile values; cheap (text only, skips unchanged). */
  update(v: HudValues): void;
}

/** Builds the HUD + hint bar in document.body. @returns handle to update it once per frame. */
export function createHud(): Hud {
  const root = el('div', 'hud');
  const scene = el('div', 'hud-scene');
  const grid = el('div', 'hud-grid');
  root.append(textEl('div', 'hud-title', 'destruction3d'), scene, grid, textEl('div', 'hud-paused', '❚❚ paused'));
  const cells = new Map<keyof HudValues, HTMLElement>();
  for (const t of TILES) {
    const tile = el('div', t.accent ? 'hud-stat accent' : 'hud-stat'), b = el('b', '');
    tile.append(b, textEl('span', '', t.label));
    grid.append(tile);
    cells.set(t.key, b);
  }
  const hints = el('div', 'hints');
  hints.innerHTML = '<span><kbd>Click</kbd> / <kbd>Space</kbd> throw ball</span><span><kbd>Shift</kbd>+<kbd>Click</kbd> remove chunk</span>'
    + '<span><kbd>Drag</kbd> orbit</span><span><kbd>P</kbd> pause</span><span><kbd>R</kbd> reset</span>';
  document.body.append(root, hints);
  return {
    setScene: (name) => { scene.textContent = name; },
    setPaused: (p) => { root.classList.toggle('paused', p); },
    update: (v) => {
      for (const [k, b] of cells) {
        const s = String(v[k]);
        if (b.textContent !== s) b.textContent = s;
      }
    },
  };
}

/** Element with a class name. */
function el(tag: string, cls: string): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  return e;
}

/** Element with a class name and text. */
function textEl(tag: string, cls: string, text: string): HTMLElement {
  const e = el(tag, cls);
  e.textContent = text;
  return e;
}
