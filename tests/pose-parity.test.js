import { describe, it, expect } from 'vitest';
import { GravityCameraRig as OptimizedRig } from '../src/core/rig.js';
import { GravityCameraRig as BaselineRig } from '../src/core/rigBaseline.js';
import { allScenarios } from '../src/sim/scenarios.js';
import { quatAngle } from '../src/core/quat.js';

/**
 * The optimised rig must reproduce the frozen baseline pose for pose. These
 * tests execute BOTH solvers on the same scenario inputs — no stubs, no
 * recorded fixtures — and assert the pose bases and quaternions are identical,
 * so a speed win can never come from silently changing the pose.
 */
function runBoth(scenario) {
  const base = new BaselineRig({
    seedUp: scenario.gravityUpAt(0),
    allowPoleCrossing: scenario.allowPoleCrossing ?? false,
  });
  const opt = new OptimizedRig({
    seedUp: scenario.gravityUpAt(0),
    allowPoleCrossing: scenario.allowPoleCrossing ?? false,
  });
  base.setYawPitch(0, 0);
  opt.setYawPitch(0, 0);
  base.resetContinuity();
  opt.resetContinuity();

  const rows = [];
  for (let i = 0; i < scenario.frames; i += 1) {
    const t = i / (scenario.frames - 1);
    const gravityUp = scenario.gravityUpAt(t);
    const look = scenario.lookAt(t, i);
    const prevUp = i > 0 ? scenario.gravityUpAt((i - 1) / (scenario.frames - 1)) : gravityUp;
    const jumped = scenario.discontinuous && i > 0
      && (prevUp.x * gravityUp.x + prevUp.y * gravityUp.y + prevUp.z * gravityUp.z) < 0.99999;
    const apply = (rig) => (jumped
      ? rig.switchGravity(gravityUp)
      : rig.step({
        gravityUp,
        dYaw: look.dYaw,
        dPitch: look.dPitch,
        allowPoleCrossing: scenario.allowPoleCrossing,
      }));
    rows.push({ i, base: apply(base), opt: apply(opt) });
  }
  return rows;
}

const exact = (a, b) => a.x === b.x && a.y === b.y && a.z === b.z;

describe('pose-update optimisation — parity with the frozen baseline', () => {
  it('produces bit-identical bases and quaternions for every scenario frame', () => {
    for (const scenario of allScenarios()) {
      for (const { i, base, opt } of runBoth(scenario)) {
        const where = `${scenario.id} frame ${i}`;
        expect(exact(base.forward, opt.forward), `${where} forward`).toBe(true);
        expect(exact(base.up, opt.up), `${where} up`).toBe(true);
        expect(exact(base.right, opt.right), `${where} right`).toBe(true);
        expect(exact(base.quaternion, opt.quaternion), `${where} quaternion`).toBe(true);
        expect(opt.yawDeg, `${where} yaw`).toBe(base.yawDeg);
        expect(opt.pitchDeg, `${where} pitch`).toBe(base.pitchDeg);
        expect(opt.solveMode, `${where} mode`).toBe(base.solveMode);
      }
    }
  });

  it('matches the baseline step, roll, orthogonality and frustum metrics', () => {
    for (const scenario of allScenarios()) {
      for (const { i, base, opt } of runBoth(scenario)) {
        const where = `${scenario.id} frame ${i}`;
        expect(opt.orthoErrorDeg, `${where} ortho`).toBe(base.orthoErrorDeg);
        expect(opt.stepDeg, `${where} step`).toBe(base.stepDeg);
        expect(opt.forwardStepDeg, `${where} forwardStep`).toBe(base.forwardStepDeg);
        expect(opt.upStepDeg, `${where} upStep`).toBe(base.upStepDeg);
        expect(opt.rightStepDeg, `${where} rightStep`).toBe(base.rightStepDeg);
        expect(opt.fovOverlap, `${where} overlap`).toBe(base.fovOverlap);
        expect(opt.rollDeg, `${where} roll`).toBe(base.rollDeg);
        expect(opt.horizonInverted, `${where} inverted`).toBe(base.horizonInverted);
        expect(opt.anomalies.map((a) => a.code), `${where} anomalies`)
          .toEqual(base.anomalies.map((a) => a.code));
      }
    }
  });

  it('keeps the degenerate pole band byte-for-byte (transport/blend untouched)', () => {
    const dwell = allScenarios().find((s) => s.id === 'pole-dwell');
    const rows = runBoth(dwell);
    const modes = new Set(rows.map((r) => r.opt.solveMode));
    expect(modes.has('transport')).toBe(true);
    for (const { i, base, opt } of rows) {
      expect(opt.sinHorizon, `frame ${i} sinHorizon`).toBe(base.sinHorizon);
      expect(opt.blendWeight, `frame ${i} blendWeight`).toBe(base.blendWeight);
    }
  });

  it('still re-solves when the cache is not reusable (gravity or look changes)', () => {
    const opt = new OptimizedRig({ seedUp: { x: 0, y: 1, z: 0 }, allowPoleCrossing: true });
    opt.setYawPitch(0.3, 0.2);
    opt.resetContinuity();
    // Same exact input twice -> a hit is allowed, but the pose must be identical.
    const a = opt.step({ gravityUp: { x: 0, y: 1, z: 0 }, dYaw: 0, dPitch: 0 });
    const b = opt.step({ gravityUp: { x: 0, y: 1, z: 0 }, dYaw: 0, dPitch: 0 });
    expect(quatAngle(a.quaternion, b.quaternion)).toBe(0);
    expect(b.stepDeg).toBe(0);
    // A single ulp of gravity change must take the full solve path.
    const tiny = { x: 0, y: 1, z: 1e-12 };
    const c = opt.step({ gravityUp: tiny, dYaw: 0, dPitch: 0 });
    expect(c.yawDeg).toBe(b.yawDeg);
    expect(opt.cacheStats.misses).toBeGreaterThanOrEqual(2);
  });
});
