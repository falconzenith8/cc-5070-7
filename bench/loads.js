/**
 * Shared load definitions for the pose-update benchmark and parity check.
 *
 * Every clip is a deterministic list of per-frame inputs. The benchmark feeds
 * the exact same clip to the frozen baseline rig (`src/core/rigBaseline.js`)
 * and to the optimised rig (`src/core/rig.js`), so any timing or angle
 * difference comes from the solver and nothing else.
 */
import { normalize, v3 } from '../src/core/vec3.js';

/** Rotate `v` about `axis` without re-normalising, so consecutive ups differ
 * by a genuine, tiny, bitwise-distinct amount (no snapping to a cached value). */
function rotate(v, axis, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const kx = axis.x;
  const ky = axis.y;
  const kz = axis.z;
  const kvx = ky * v.z - kz * v.y;
  const kvy = kz * v.x - kx * v.z;
  const kvz = kx * v.y - ky * v.x;
  const d = kx * v.x + ky * v.y + kz * v.z;
  return {
    x: v.x * c + kvx * s + kx * d * (1 - c),
    y: v.y * c + kvy * s + ky * d * (1 - c),
    z: v.z * c + kvz * s + kz * d * (1 - c),
  };
}

/**
 * Load catalogue. `seed` is the starting gravity up, `pitch` the starting
 * look elevation; `frames` is the clip length used by the benchmark.
 *
 * - slowGravity: gravity turns 0.020°/frame on a tilted axis while yaw/pitch
 *   drift. This is the "缓慢转重力" load: every frame takes the full solve.
 * - reversedAxis: gravity near -Y with pitch pinned close to the reversed
 *   pole, sweeping through the degenerate band. This is the "反向轴" load.
 * - slowGravityIdleLook: gravity still turns every frame but the look input is
 *   constant — the gravity basis cache cannot be reused, so this isolates the
 *   geometry/metric work from the input cache.
 * - idle: gravity and look input are bit-identical every frame, so the exact
 *   cache is allowed to return the previous solve. This is the "unchanged
 *   frame" case the cache exists for.
 */
export const LOADS = [
  {
    id: 'slow-gravity',
    seed: v3(0, 1, 0),
    pitchDeg: 20,
    frames: 12000,
    clip: (frames) => {
      const axis = normalize(v3(0.3, 0.05, 0.95));
      const out = [];
      for (let i = 0; i < frames; i += 1) {
        out.push({ gravityUp: rotate(v3(0, 1, 0), axis, i * 0.00035), dYaw: 0.00012, dPitch: 0.00006 });
      }
      return out;
    },
  },
  {
    id: 'reversed-axis',
    seed: normalize(v3(0, -1, 0.0005)),
    pitchDeg: 89.2,
    frames: 12000,
    clip: (frames) => {
      const axis = normalize(v3(0.15, 0.05, 0.99));
      const out = [];
      for (let i = 0; i < frames; i += 1) {
        out.push({
          gravityUp: rotate(normalize(v3(0, -1, 0.0005)), axis, Math.sin(i * 0.0006) * 0.012),
          dYaw: 0.00025,
          dPitch: 0.00002,
        });
      }
      return out;
    },
  },
  {
    id: 'slow-gravity-idle-look',
    seed: v3(0, 1, 0),
    pitchDeg: 30,
    frames: 12000,
    clip: (frames) => {
      const axis = normalize(v3(0.25, 0.1, 0.96));
      const out = [];
      for (let i = 0; i < frames; i += 1) {
        out.push({ gravityUp: rotate(v3(0, 1, 0), axis, i * 0.00012), dYaw: 0, dPitch: 0 });
      }
      return out;
    },
  },
  {
    id: 'idle',
    seed: v3(0, 1, 0),
    pitchDeg: 10,
    frames: 12000,
    clip: (frames) => {
      const out = [];
      for (let i = 0; i < frames; i += 1) out.push({ gravityUp: v3(0, 1, 0), dYaw: 0, dPitch: 0 });
      return out;
    },
  },
];

/** Run one clip through a rig class and return timings plus angle/step metrics. */
export function runClip(RigClass, load, clip) {
  const rig = new RigClass({ seedUp: load.seed, allowPoleCrossing: true, fps: 60 });
  rig.setYawPitch(0.2, (load.pitchDeg * Math.PI) / 180);
  rig.resetContinuity();
  let maxStepDeg = 0;
  let maxOrthoErrorDeg = 0;
  let maxRollDeg = 0;
  let transport = 0;
  let blend = 0;
  const t0 = process.hrtime.bigint();
  for (const frame of clip) {
    const pose = rig.step(frame);
    if (pose.stepDeg > maxStepDeg) maxStepDeg = pose.stepDeg;
    if (pose.orthoErrorDeg > maxOrthoErrorDeg) maxOrthoErrorDeg = pose.orthoErrorDeg;
    if (pose.rollDeg != null) maxRollDeg = Math.max(maxRollDeg, Math.abs(pose.rollDeg));
    if (pose.solveMode === 'transport') transport += 1;
    if (pose.solveMode === 'blend') blend += 1;
  }
  const t1 = process.hrtime.bigint();
  const usPerFrame = Number(t1 - t0) / 1000 / clip.length;
  const finalPose = rig.lastPose;
  return {
    rig,
    usPerFrame,
    maxStepDeg,
    maxOrthoErrorDeg,
    maxRollDeg,
    transport,
    blend,
    finalPose,
  };
}

export function median(values) {
  const s = [...values].sort((a, b) => a - b);
  return s[Math.floor(s.length / 2)];
}
