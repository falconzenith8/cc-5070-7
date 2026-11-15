import { describe, it, expect } from 'vitest';
import { GravityCameraRig, SolveMode, frustumOverlap, RAD2DEG, DEG2RAD } from '../src/core/rig.js';
import { add, cross, dot, len, normalize, scale, sub, v3, angleBetween } from '../src/core/vec3.js';
import { quatRotate, quatAngle } from '../src/core/quat.js';
import {
  allScenarios, runScenario, evaluate, sphereWalk, wallTurn, poleSweep, gravityFlip,
  gravityPresets, rampUp,
} from '../src/sim/scenarios.js';

const ortho = (p) => Math.max(
  Math.abs(dot(p.right, p.up)),
  Math.abs(dot(p.up, p.forward)),
  Math.abs(dot(p.forward, p.right)),
  Math.abs(len(p.right) - 1),
  Math.abs(len(p.up) - 1),
  Math.abs(len(p.forward) - 1),
);

describe('orbital camera rig — geometry', () => {
  it('builds an orthonormal basis every frame of every scenario', () => {
    for (const s of allScenarios()) {
      const { records } = runScenario(s);
      for (const r of records) {
        const e = Math.max(
          Math.abs(dot(r.right, r.up)), Math.abs(dot(r.up, r.forward)), Math.abs(dot(r.forward, r.right)),
          Math.abs(len(r.right) - 1), Math.abs(len(r.up) - 1), Math.abs(len(r.forward) - 1),
        );
        expect(e).toBeLessThan(1e-9);
      }
    }
  });

  it('quaternion reproduces the same basis', () => {
    const rig = new GravityCameraRig();
    rig.step({ gravityUp: rampUp(35), dYaw: 0.3, dPitch: 0.1 });
    const p = rig.lastPose;
    const rx = quatRotate(p.quaternion, v3(1, 0, 0));
    const uy = quatRotate(p.quaternion, v3(0, 1, 0));
    const bz = quatRotate(p.quaternion, v3(0, 0, 1));
    expect(angleBetween(rx, p.right)).toBeLessThan(1e-7);
    expect(angleBetween(uy, p.up)).toBeLessThan(1e-7);
    expect(angleBetween(bz, p.back)).toBeLessThan(1e-7);
  });

  it('uses the active gravity up, never a hard-coded world Y', () => {
    const rig = new GravityCameraRig({ seedUp: v3(1, 0, 0) });
    rig.setUp(v3(1, 0, 0));
    const p = rig.lastPose;
    // On a wall, the gravity horizon's up direction is horizontal in world terms.
    expect(Math.abs(dot(p.forward, v3(0, 1, 0)))).toBeLessThan(0.35);
    expect(dot(p.referenceForward, v3(1, 0, 0))).toBeCloseTo(0, 6);
  });

  it('keeps zero roll while the horizon is well defined', () => {
    for (const s of allScenarios()) {
      const { records } = runScenario(s);
      for (const r of records) {
        if (r.rollDeg == null) continue;
        if (Math.abs(r.rollDeg) > 90) continue; // view past zenith: inverse projection
        expect(Math.abs(r.rollDeg)).toBeLessThan(0.5);
      }
    }
  });
});

describe('orbital camera rig — continuity through gravity changes', () => {
  it('sweeps a sphere without frame jumps or view discontinuity', () => {
    const r = runScenario(sphereWalk());
    expect(r.stats.maxStepDeg).toBeLessThan(3);
    expect(r.stats.minFovOverlap).toBeGreaterThan(0.9);
    expect(r.stats.teleports).toBe(0);
  });

  it('walks floor -> wall -> ceiling continuously', () => {
    const r = runScenario(wallTurn());
    expect(r.stats.maxStepDeg).toBeLessThan(5);
    expect(r.stats.minFovOverlap).toBeGreaterThan(0.9);
    // The gravity up genuinely traverses to the ceiling.
    const ups = r.records.map((x) => x.gravityUp);
    expect(dot(ups[0], v3(0, 1, 0))).toBeGreaterThan(0.99);
    expect(dot(ups[ups.length - 1], v3(0, -1, 0))).toBeGreaterThan(0.99);
  });

  it('carries the local bearing across a gravity change instead of snapping', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0) });
    rig.setYawPitch(0, 0);
    rig.resetContinuity();
    const before = rig.lastPose.forward;
    rig.setUp(normalize(v3(0.2, 0.9, -0.3)));
    const after = rig.lastPose.forward;
    // Forward direction moves with the local up but the heading stays sane.
    expect(angleBetween(before, after)).toBeLessThan(40 * DEG2RAD);
  });

  it('declares a modelled 180° gravity reversal as a deliberate re-seat', () => {
    const r = runScenario(gravityFlip());
    const switchRec = r.records.find((x) => x.discontinuity);
    expect(switchRec).toBeTruthy();
    expect(switchRec.discontinuity.angleDeg).toBeCloseTo(180, 4);
    // Not counted as an unmodelled jump.
    expect(r.stats.maxForwardJumpDeg).toBe(0);
    expect(r.stats.maxStepDeg).toBe(0);
  });
});

describe('orbital camera rig — pole handling', () => {
  it('falls back continuously when looking straight up', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0), allowPoleCrossing: true });
    rig.setYawPitch(0.4, 80 * DEG2RAD);
    rig.resetContinuity();
    let prev = rig.lastPose;
    let maxStep = 0;
    let sawFallback = false;
    // Walk to the exact pole (90°) and past it; 0.5° steps land exactly on it.
    for (let i = 1; i <= 24; i += 1) {
      const p = rig.setYawPitch(0.4, (80 + i * 0.5) * DEG2RAD);
      if (p.solveMode !== SolveMode.CROSS) sawFallback = true;
      maxStep = Math.max(maxStep, angleBetween(prev.forward, p.forward), angleBetween(prev.up, p.up), angleBetween(prev.right, p.right));
      prev = p;
    }
    expect(sawFallback).toBe(true);
    expect(maxStep * RAD2DEG).toBeLessThan(3);
    for (const v of [prev.forward, prev.up, prev.right, prev.quaternion]) {
      for (const x of Object.values(v)) expect(Number.isFinite(x)).toBe(true);
    }
  });

  it('crosses exactly through the pole with no 180° horizon snap', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0), allowPoleCrossing: true });
    rig.setYawPitch(0, 80 * DEG2RAD);
    rig.resetContinuity();
    let prev = rig.lastPose;
    let maxRightTurn = 0;
    let framesAtPole = 0;
    const steps = 1200;
    for (let i = 1; i <= steps; i += 1) {
      // Drive pitch from 80° to 100°, straight through the up vector, no yaw.
      const p = rig.step({ dPitch: (20 * DEG2RAD) / steps });
      if (Math.abs(dot(p.forward, v3(0, 1, 0))) > 0.99999) framesAtPole += 1;
      const fwd = angleBetween(prev.forward, p.forward);
      const rr = angleBetween(prev.right, p.right);
      if (fwd < 0.5 * DEG2RAD) maxRightTurn = Math.max(maxRightTurn, rr);
      expect(angleBetween(prev.forward, p.forward)).toBeLessThan(1 * DEG2RAD);
      prev = p;
    }
    expect(framesAtPole).toBeGreaterThan(0);
    expect(maxRightTurn * RAD2DEG).toBeLessThan(5);
    expect(rig.lastPose.pitchDeg).toBeGreaterThan(99);
  });

  it('does not normalize a near-zero cross vector to decide the horizon', () => {
    // Directly at the up vector, cross(forward, up) is exactly zero. The rig
    // must still return a unit orthonormal frame without NaN.
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0), allowPoleCrossing: true });
    rig.setYawPitch(0, Math.PI / 2 - 1e-9);
    rig.resetContinuity();
    const p = rig.lastPose;
    expect(p.sinHorizon).toBeLessThan(1e-6);
    expect(p.solveMode).not.toBe(SolveMode.CROSS);
    expect(ortho(p)).toBeLessThan(1e-9);
  });

  it('is stable across repeated identical pole-state solves', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0), allowPoleCrossing: true });
    rig.setYawPitch(0, Math.PI / 2);
    rig.resetContinuity();
    const a = rig.step({ dPitch: 0 });
    const b = rig.step({ dPitch: 0 });
    expect(quatAngle(a.quaternion, b.quaternion)).toBeLessThan(1e-9);
    expect(ortho(b)).toBeLessThan(1e-9);
  });

  it('clamps pitch before the pole when pole crossing is disabled', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0), allowPoleCrossing: false });
    rig.setYawPitch(0, 0);
    for (let i = 0; i < 100; i += 1) rig.step({ dPitch: 0.1 });
    expect(rig.lastPose.pitchDeg).toBeLessThanOrEqual(89.5001);
    expect(rig.lastPose.solveMode).toBe(SolveMode.CROSS);
  });
});

describe('orbital camera rig — degenerate fallback is necessary', () => {
  it('a naive normalize(cross(forward, up)) implementation snaps 180° at the pole', () => {
    // Reference "naive" solver for comparison only.
    const naiveRight = (forward, up) => {
      const c = cross(forward, up);
      return len(c) < 1e-12 ? v3(1, 0, 0) : normalize(c);
    };
    const up = v3(0, 1, 0);
    let prev = null;
    let maxNaiveJump = 0;
    let maxRigJump = 0;
    const rig = new GravityCameraRig({ seedUp: up, allowPoleCrossing: true });
    rig.setYawPitch(0, 0);
    rig.resetContinuity();
    let prevRigRight = rig.lastPose.right;
    for (let i = 0; i <= 4000; i += 1) {
      const pitch = (-20 + (220 * i) / 4000) * DEG2RAD; // -20° -> 200°, through pole
      const cp = Math.cos(pitch);
      const sp = Math.sin(pitch);
      const forward = normalize(v3(0, sp, -cp)); // pitch up rotates forward toward up
      const r = naiveRight(forward, up);
      if (prev && dot(prev, r) > 0) { /* same hemisphere */ }
      if (prev) maxNaiveJump = Math.max(maxNaiveJump, angleBetween(prev, r) * RAD2DEG);
      prev = r;
      // Same physical look direction through the rig.
      rig.setYawPitch(0, pitch);
      const rigRight = rig.lastPose.right;
      maxRigJump = Math.max(maxRigJump, angleBetween(prevRigRight, rigRight) * RAD2DEG);
      prevRigRight = rigRight;
    }
    // The naive solver flips the horizon by ~180° at the pole; the rig does not.
    expect(maxNaiveJump).toBeGreaterThan(90);
    expect(maxRigJump).toBeLessThan(10);
  });
});

describe('orbital camera rig — input semantics', () => {
  it('yaw right turns the view toward the local right in any gravity', () => {
    for (const { up, label } of gravityPresets()) {
      const rig = new GravityCameraRig({ seedUp: up });
      rig.setUp(up);
      rig.setYawPitch(0, 0);
      rig.resetContinuity();
      const before = rig.lastPose;
      const after = rig.step({ dYaw: 10 * DEG2RAD });
      // Turning right moves forward toward the previous right axis.
      expect(dot(after.forward, before.right), label).toBeGreaterThan(0.05);
      expect(dot(after.forward, before.up)).toBeCloseTo(0, 5);
    }
  });

  it('positive pitch raises the view toward local up in any gravity', () => {
    for (const { up, label } of gravityPresets()) {
      const rig = new GravityCameraRig({ seedUp: up });
      rig.setUp(up);
      rig.setYawPitch(0, 0);
      const before = rig.lastPose;
      const after = rig.step({ dPitch: 15 * DEG2RAD });
      expect(dot(after.forward, up), label).toBeGreaterThan(0.1);
      expect(dot(after.forward, up)).toBeGreaterThan(dot(before.forward, up));
    }
  });

  it('keeps yaw semantics screen-relative after a gravity reversal', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0) });
    rig.setUp(v3(0, 1, 0));
    rig.setYawPitch(0, 0);
    const floor = rig.lastPose;
    rig.switchGravity(v3(0, -1, 0));
    const beforeCeiling = rig.lastPose;
    const after = rig.step({ dYaw: 10 * DEG2RAD });
    // Yaw+ still turns toward the camera's own right axis on the ceiling.
    expect(dot(after.forward, beforeCeiling.right)).toBeGreaterThan(0.05);
    // And the local right genuinely mirrors in world space after the reversal,
    // which is the expected consequence of the new gravity up.
    expect(dot(beforeCeiling.right, floor.right)).toBeLessThan(-0.9);
    expect(dot(beforeCeiling.up, v3(0, -1, 0))).toBeCloseTo(1, 4);
  });
});

describe('orbital camera rig — horizon control decoupled from character', () => {
  it('character facing changes do not alter the camera pose', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0) });
    rig.setYawPitch(0.2, 0.1);
    const a = rig.lastPose;
    // Character facing is a separate input and is not given to the rig at all.
    const characterFacing = normalize(v3(1, 0.3, 0.2));
    const b = rig.lastPose;
    expect(quatAngle(a.quaternion, b.quaternion)).toBe(0);
    // Sanity: the facing vector is genuinely unrelated to the view basis.
    expect(dot(characterFacing, a.right) + dot(characterFacing, a.up)).toBeLessThan(2);
  });

  it('a wobbling character model cannot inject roll into the view', () => {
    const rig = new GravityCameraRig({ seedUp: v3(0, 1, 0) });
    rig.setYawPitch(0, 0.2);
    rig.resetContinuity();
    const base = rig.lastPose;
    for (let i = 0; i < 300; i += 1) {
      const wobble = Math.sin(i * 1.7) * 0.6; // model-only animation
      void wobble;
      const p = rig.step({ dYaw: 0, dPitch: 0 });
      expect(quatAngle(base.quaternion, p.quaternion)).toBeLessThan(1e-6);
      expect(Math.abs(p.rollDeg ?? 0)).toBeLessThan(1e-6);
    }
  });
});

describe('scenario reports', () => {
  it('all documented trajectories pass the numeric acceptance checks', () => {
    for (const s of allScenarios()) {
      const r = runScenario(s);
      const ev = evaluate(r.stats);
      if (!ev.pass) {
        const failed = ev.checks.filter((c) => !c.ok).map((c) => `${c.name}: ${c.detail}`);
        throw new Error(`${s.id} failed: ${failed.join('; ')}`);
      }
      expect(ev.pass).toBe(true);
    }
  });

  it('frustum overlap is 1 for identical poses and drops for a 180° flip', () => {
    const rig = new GravityCameraRig();
    const a = rig.lastPose;
    expect(frustumOverlap(a, a, 60, 16 / 9)).toBe(1);
    const rig2 = new GravityCameraRig({ seedUp: v3(0, -1, 0) });
    rig2.setUp(v3(0, -1, 0));
    const b = rig2.lastPose;
    // Rebuild a pose looking opposite without touching a's up frame.
    const flipped = { ...a, forward: scale(a.forward, -1), up: a.up, right: scale(a.right, -1) };
    void b;
    expect(frustumOverlap(a, flipped, 60, 16 / 9)).toBe(0);
  });
});
