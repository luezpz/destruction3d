import { describe, expect, it } from 'vitest';
import { MATERIAL_PRESETS, UNIT_SCALE, createMaterialTable, scaleMaterials } from '../src/model/Material';

describe('scaleMaterials', () => {
  it('unit scale is identity and does not mutate presets', () => {
    expect(scaleMaterials(MATERIAL_PRESETS, UNIT_SCALE)).toEqual(MATERIAL_PRESETS);
  });
  it('scales strengths globally and per material, and stays valid', () => {
    const s = scaleMaterials(MATERIAL_PRESETS, { ...UNIT_SCALE, strength: 2, stiffness: 3 }, { concrete: 5 });
    const c0 = MATERIAL_PRESETS.find((m) => m.name === 'concrete')!, c = s.find((m) => m.name === 'concrete')!;
    const b0 = MATERIAL_PRESETS.find((m) => m.name === 'brick')!, b = s.find((m) => m.name === 'brick')!;
    expect(c.sigmaT).toBeCloseTo(c0.sigmaT * 10);
    expect(b.tauC).toBeCloseTo(b0.tauC * 2);
    expect(b.E).toBeCloseTo(b0.E * 3);
    expect(c0.sigmaT).toBe(3);
    expect(() => createMaterialTable(s)).not.toThrow();
  });
});
