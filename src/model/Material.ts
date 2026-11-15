// Data-driven material records (PLAN §2b). Code must branch on fields, never on names.
// Units: density kg/m^3, E/H in GPa, strengths in MPa (converted to Pa only inside kernels).

export type DeformMode = 'none' | 'offsets' | 'joints';
export type HullMode = 'rigid' | 'refit';

export interface Material {
  name: string;
  density: number;
  E: number; // GPa
  nu: number;
  sigmaT: number; // MPa tension
  sigmaC: number; // MPa compression; 0 -> use sigmaT
  tauC: number; // MPa cohesion (shear)
  mu: number; // friction coefficient for shear
  sigmaY: number; // MPa yield; 0 = no real plasticity
  H: number; // GPa hardening (sigmaY > 0 only)
  epsPlasticMax: number; // ductile break strain (sigmaY > 0 only)
  epsElasticMax: number; // elastic break strain (sigmaY == 0 only)
  damageRate: number; // per second per unit overload; Infinity = instant
  filterTime: number; // s, ratio low-pass window
  propagation: number; // damage added to neighbour bonds on a break
  grain: { axis: [number, number, number]; strengthRatio: number } | null;
  deform: DeformMode;
  /** Fake impact dents (PLAN §4, used when deform === 'offsets'). Lengths are fractions of the chunk size (cbrt volume). */
  dent: DentParams;
  hull: HullMode;
  render: { outer: number; inner: number }; // sRGB hex colours (placeholder for render materials)
  reinforcement: { fraction: number; material: string } | null;
}

export interface DentParams {
  oMax: number; // max offset per chunk (fraction of chunk size)
  jRef: number; // N s, impulse scale of the saturation s = oMax (1 - exp(-J / jRef))
  radius: number; // bond hops of the falloff phi(g / radius)
  tilt: number; // k_t, tilt gain (rad per unit of offset/radius lever)
  wMax: number; // offset difference across a bond (fraction of chunk size) that breaks it: d_b = w_b / wMax
  minForce: number; // N, contact forces below this never dent (resting piles would creep otherwise)
}

const base = {
  dent: { oMax: 0.08, jRef: 2000, radius: 2, tilt: 0.5, wMax: 0.035, minForce: 2e4 } as DentParams,
  sigmaY: 0, H: 0, epsPlasticMax: 0, grain: null, hull: 'rigid', reinforcement: null, filterTime: 0.05, propagation: 0,
} as const;

/** Preset records. Values are order-of-magnitude engineering figures, tuned for gameplay later. */
export const MATERIAL_PRESETS: Material[] = [
  { ...base, name: 'concrete', density: 2400, E: 30, nu: 0.2, sigmaT: 3, sigmaC: 30, tauC: 4, mu: 0.6, epsElasticMax: 0.0035, damageRate: 5, deform: 'offsets', render: { outer: 0x9a9a95, inner: 0x6d6a63 } },
  { ...base, name: 'reinforcedConcrete', density: 2500, E: 30, nu: 0.2, sigmaT: 3, sigmaC: 30, tauC: 4, mu: 0.6, epsElasticMax: 0.0035, damageRate: 5, deform: 'offsets', render: { outer: 0x8f908c, inner: 0x6d6a63 }, reinforcement: { fraction: 0.02, material: 'steel' } },
  { ...base, name: 'brick', density: 1900, E: 15, nu: 0.2, sigmaT: 2, sigmaC: 20, tauC: 2, mu: 0.6, epsElasticMax: 0.003, damageRate: 5, deform: 'offsets', render: { outer: 0xa0472f, inner: 0x7a3a28 } },
  { ...base, name: 'mortar', density: 1800, E: 5, nu: 0.2, sigmaT: 0.5, sigmaC: 8, tauC: 0.5, mu: 0.6, epsElasticMax: 0.003, damageRate: 8, deform: 'none', render: { outer: 0xc8c2b4, inner: 0xb0a998 } },
  { ...base, name: 'glass', density: 2500, E: 70, nu: 0.22, sigmaT: 40, sigmaC: 400, tauC: 30, mu: 0.3, epsElasticMax: 0.001, damageRate: Infinity, propagation: 0.6, deform: 'none', render: { outer: 0xbfe6ff, inner: 0xe6f6ff } },
  { ...base, name: 'ice', density: 917, E: 9, nu: 0.33, sigmaT: 1, sigmaC: 5, tauC: 0.7, mu: 0.1, epsElasticMax: 0.0005, damageRate: Infinity, propagation: 0.3, deform: 'none', render: { outer: 0xdaf0ff, inner: 0xf2faff } },
  { ...base, name: 'wood', density: 600, E: 11, nu: 0.3, sigmaT: 40, sigmaC: 30, tauC: 6, mu: 0.4, epsElasticMax: 0.01, damageRate: 3, deform: 'offsets', grain: { axis: [0, 1, 0], strengthRatio: 0.1 }, render: { outer: 0x9b6b3d, inner: 0xd2a76b } },
  { ...base, name: 'drywall', density: 700, E: 2, nu: 0.25, sigmaT: 0.5, sigmaC: 3, tauC: 0.5, mu: 0.4, epsElasticMax: 0.004, damageRate: 20, deform: 'none', render: { outer: 0xeeeeea, inner: 0xd8d4c8 } },
  { ...base, name: 'steel', density: 7850, E: 200, nu: 0.3, sigmaT: 400, sigmaC: 400, tauC: 250, mu: 0.3, sigmaY: 250, H: 2, epsPlasticMax: 0.2, epsElasticMax: 0, damageRate: 2, deform: 'joints', render: { outer: 0x7c858c, inner: 0xb8c0c6 } },
  { ...base, name: 'aluminium', density: 2700, E: 69, nu: 0.33, sigmaT: 300, sigmaC: 300, tauC: 200, mu: 0.3, sigmaY: 150, H: 1, epsPlasticMax: 0.12, epsElasticMax: 0, damageRate: 2, deform: 'joints', hull: 'refit', render: { outer: 0xc3c7cb, inner: 0xe0e3e6 } },
  { ...base, name: 'rubber', density: 1100, E: 0.01, nu: 0.49, sigmaT: 15, sigmaC: 15, tauC: 10, mu: 0.9, epsElasticMax: 5, damageRate: 1, deform: 'joints', render: { outer: 0x222222, inner: 0x333333 } },
  { ...base, name: 'clay', density: 1800, E: 0.05, nu: 0.4, sigmaT: 0.05, sigmaC: 0.2, tauC: 0.05, mu: 0.3, sigmaY: 0.02, H: 0, epsPlasticMax: 0.5, epsElasticMax: 0, damageRate: 1, deform: 'offsets', dent: { oMax: 0.25, jRef: 50, radius: 2, tilt: 0.5, wMax: 0.3, minForce: 500 }, render: { outer: 0xb5764f, inner: 0x9c623f } },
];

/** Global debug multipliers applied to every record (tuning knobs; 1 = preset values). */
export interface MaterialScale {
  strength: number; // sigmaT, sigmaC, tauC, sigmaY
  stiffness: number; // E, H
  ductility: number; // epsElasticMax, epsPlasticMax
  damageRate: number;
  density: number;
}

export const UNIT_SCALE: MaterialScale = { strength: 1, stiffness: 1, ductility: 1, damageRate: 1, density: 1 };

/**
 * Copies records with multipliers applied. @param ms records; @param s global scale; @param perStrength optional
 * extra strength multiplier by material name (tuning only, not a name branch in sim code). @returns new records.
 */
export function scaleMaterials(ms: Material[], s: MaterialScale, perStrength: Record<string, number> = {}): Material[] {
  return ms.map((m) => {
    const k = s.strength * (perStrength[m.name] ?? 1);
    return {
      ...m,
      sigmaT: m.sigmaT * k, sigmaC: m.sigmaC * k, tauC: m.tauC * k, sigmaY: m.sigmaY * k,
      E: m.E * s.stiffness, H: m.H * s.stiffness,
      epsElasticMax: m.epsElasticMax * s.ductility, epsPlasticMax: m.epsPlasticMax * s.ductility,
      damageRate: m.damageRate * s.damageRate, density: m.density * s.density,
    };
  });
}

/** Lookup table of materials by id (array index) and name. */
export interface MaterialTable {
  materials: Material[];
  idByName: Map<string, number>;
}

/** Returns a list of validation errors for one record (empty = valid). */
export function validateMaterial(m: Material, names: ReadonlySet<string>): string[] {
  const errs: string[] = [];
  const pos = (k: keyof Material) => { if (!((m[k] as number) > 0)) errs.push(`${m.name}: ${k} must be > 0`); };
  pos('density'); pos('E'); pos('sigmaT'); pos('tauC'); pos('filterTime');
  if (!(m.nu >= 0 && m.nu < 0.5)) errs.push(`${m.name}: nu must be in [0, 0.5)`);
  if (m.sigmaC < 0 || m.mu < 0 || m.sigmaY < 0 || m.H < 0 || m.propagation < 0) errs.push(`${m.name}: negative parameter`);
  if (!(m.damageRate > 0)) errs.push(`${m.name}: damageRate must be > 0 (Infinity allowed)`);
  if (m.sigmaY > 0 && !(m.epsPlasticMax > 0)) errs.push(`${m.name}: plastic material needs epsPlasticMax > 0`);
  if (m.sigmaY === 0 && !(m.epsElasticMax > 0)) errs.push(`${m.name}: non-plastic material needs epsElasticMax > 0`);
  if (m.hull === 'refit' && m.deform !== 'joints') errs.push(`${m.name}: hull 'refit' requires deform 'joints'`);
  const d = m.dent;
  if (!d || !(d.oMax > 0 && d.jRef > 0 && d.radius >= 1 && d.tilt >= 0 && d.wMax > 0 && d.minForce >= 0)) errs.push(`${m.name}: dent needs oMax, jRef, wMax > 0, radius >= 1, tilt, minForce >= 0`);
  if (m.grain && !(m.grain.strengthRatio > 0 && m.grain.strengthRatio <= 1)) errs.push(`${m.name}: grain.strengthRatio must be in (0, 1]`);
  if (m.reinforcement) {
    if (!names.has(m.reinforcement.material)) errs.push(`${m.name}: unknown reinforcement material '${m.reinforcement.material}'`);
    if (!(m.reinforcement.fraction > 0 && m.reinforcement.fraction < 1)) errs.push(`${m.name}: reinforcement.fraction must be in (0, 1)`);
  }
  return errs;
}

/** Builds a table, throwing on duplicate names or invalid records (invalid combos rejected at load, PLAN §2b). */
export function createMaterialTable(materials: Material[] = MATERIAL_PRESETS): MaterialTable {
  const idByName = new Map<string, number>();
  materials.forEach((m, i) => {
    if (idByName.has(m.name)) throw new Error(`duplicate material '${m.name}'`);
    idByName.set(m.name, i);
  });
  const names = new Set(idByName.keys());
  const errs = materials.flatMap((m) => validateMaterial(m, names));
  if (errs.length) throw new Error(`invalid materials:\n${errs.join('\n')}`);
  if (materials.length >= 0xffff) throw new Error('too many materials for Uint16 ids');
  return { materials: materials.slice(), idByName }; // copy: interfaceMaterial appends derived records
}

/** Material id by name; throws if missing. */
export function materialId(table: MaterialTable, name: string): number {
  const id = table.idByName.get(name);
  if (id === undefined) throw new Error(`unknown material '${name}'`);
  return id;
}

/**
 * Derived interface material for a bond between two different materials (no explicit interface given):
 * stiffness in series 1/E = 1/E_a + 1/E_b (GPa), and the weaker of each strength. Appended to the table once per pair.
 * @returns id of the derived (or shared) material.
 */
export function interfaceMaterial(table: MaterialTable, a: number, b: number): number {
  if (a === b) return a;
  const [lo, hi] = a < b ? [a, b] : [b, a];
  const ma = table.materials[lo], mb = table.materials[hi];
  const name = `${ma.name}|${mb.name}`;
  const existing = table.idByName.get(name);
  if (existing !== undefined) return existing;
  const weaker = <K extends keyof Material>(k: K) => Math.min(ma[k] as number, mb[k] as number);
  const sigmaCa = ma.sigmaC || ma.sigmaT, sigmaCb = mb.sigmaC || mb.sigmaT;
  // Plasticity only if both sides are plastic; otherwise the brittle side governs.
  const plastic = ma.sigmaY > 0 && mb.sigmaY > 0;
  const m: Material = {
    ...ma,
    name,
    density: 0.5 * (ma.density + mb.density),
    E: 1 / (1 / ma.E + 1 / mb.E),
    nu: 0.5 * (ma.nu + mb.nu),
    sigmaT: weaker('sigmaT'), sigmaC: Math.min(sigmaCa, sigmaCb), tauC: weaker('tauC'), mu: weaker('mu'),
    sigmaY: plastic ? weaker('sigmaY') : 0,
    H: plastic ? weaker('H') : 0,
    epsPlasticMax: plastic ? weaker('epsPlasticMax') : 0,
    epsElasticMax: plastic ? 0 : Math.min(...[ma, mb].map((x) => x.epsElasticMax || Infinity)),
    damageRate: Math.max(ma.damageRate, mb.damageRate),
    filterTime: Math.min(ma.filterTime, mb.filterTime),
    propagation: Math.max(ma.propagation, mb.propagation),
    grain: null,
    deform: ma.deform === mb.deform ? ma.deform : 'none',
    hull: ma.hull === mb.hull ? ma.hull : 'rigid',
    reinforcement: null,
  };
  if (m.hull === 'refit' && m.deform !== 'joints') m.hull = 'rigid';
  const errs = validateMaterial(m, new Set(table.idByName.keys()));
  if (errs.length) throw new Error(`derived interface material invalid:\n${errs.join('\n')}`);
  table.materials.push(m);
  table.idByName.set(name, table.materials.length - 1);
  return table.materials.length - 1;
}
