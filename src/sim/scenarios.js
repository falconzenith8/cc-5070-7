import { add, cross, normalize, scale, sub, v3, len, dot } from '../core/vec3.js';
import { GravityCameraRig, SolveMode, RAD2DEG } from '../core/rig.js';

const TAU = Math.PI * 2;
const DEG = Math.PI / 180;

/**
 * Continuous gravity fields used by the demo and the tests. Every `up` here is
 * derived from an actual surface (planet radius, wall normal, ramp normal), so
 * world-up is never assumed for a region.
 */
export function planetUp(position, radius = 40, center = v3(0, 0, 0)) {
  return normalize(sub(position, center));
}

export function rampUp(angleDeg) {
  const a = angleDeg * DEG;
  return normalize(v3(-Math.sin(a), Math.cos(a), 0));
}

export function gravityPresets() {
  return [
    { id: 'G_FLOOR', label: '地面重力 +Y', up: v3(0, 1, 0), wall: 'floor' },
    { id: 'G_CEILING', label: '天花板重力 -Y', up: v3(0, -1, 0), wall: 'ceiling' },
    { id: 'G_WEST', label: '西墙重力 +X', up: v3(1, 0, 0), wall: 'west' },
    { id: 'G_EAST', label: '东墙重力 -X', up: v3(-1, 0, 0), wall: 'east' },
    { id: 'G_SLOPE', label: '斜面重力 35°', up: rampUp(35), wall: 'slope' },
    { id: 'G_TILT', label: '斜角重力 (30°,15°)', up: normalize(v3(0.5, 0.85, 0.2)), wall: 'tilt' },
  ];
}

/** Player walks a great circle on the planet; gravity follows the surface. */
export function sphereWalk({
  frames = 900,
  radius = 40,
  laps = 0.85,
  lookYaw = 18 * DEG,
  lookPitch = 8 * DEG,
  yawRate = 0,
} = {}) {
  const start = normalize(v3(0.35, 0.12, 0.93));
  const axis = normalize(v3(0.1, 1, 0.15));
  const tangent = normalize(cross(axis, start));
  return {
    id: 'sphere-walk',
    label: '球面行走',
    frames,
    gravityUpAt: (t) => {
      const a = t * TAU * laps;
      const dir = normalize(add(scale(start, Math.cos(a)), scale(tangent, Math.sin(a))));
      return dir;
    },
    playerPosAt: (t) => {
      const a = t * TAU * laps;
      return scale(add(scale(start, Math.cos(a)), scale(tangent, Math.sin(a))), radius);
    },
    lookAt: (t) => ({ dYaw: (lookYaw - 0 + yawRate * t) / frames, dPitch: lookPitch / frames }),
  };
}

/**
 * Floor -> sloped wall -> overhead surface. Each segment supplies the real
 * surface normal; interpolation between segments is a normalized lerp, which
 * corresponds to the rounded corner geometry in the chamber.
 */
export function wallTurn({ frames = 600 } = {}) {
  const normals = [
    { t: 0.0, up: normalize(v3(0, 1, 0)) },
    { t: 0.22, up: normalize(v3(0, 0.82, -0.57)) },
    { t: 0.42, up: normalize(v3(0, 0.45, -0.89)) },
    { t: 0.52, up: normalize(v3(0, 0, -1)) },
    { t: 0.72, up: normalize(v3(0, -0.35, -0.94)) },
    { t: 0.88, up: normalize(v3(0, -1, 0)) },
    { t: 1.0, up: normalize(v3(0, -1, 0)) },
  ];
  return {
    id: 'wall-turn',
    label: '墙面转向（地板→墙→顶部）',
    frames,
    gravityUpAt: (t) => sampleUp(normals, t),
    gravityRateAt: (t) => {
      // Analytic-ish rate: central difference of the sampled normal field.
      const h = 1 / (frames * 4);
      const a = sampleUp(normals, Math.min(1, t + h));
      const b = sampleUp(normals, Math.max(0, t - h));
      return Math.acos(Math.min(1, dot(a, b))) / (2 * h) * RAD2DEG;
    },
    playerPosAt: null,
    lookAt: () => ({ dYaw: (70 * DEG) / frames, dPitch: 0 }),
  };
}

function projectTangent(v, up) {
  return sub(v, scale(up, dot(v, up)));
}

function sampleUp(points, t) {
  for (let i = 1; i < points.length; i += 1) {
    if (t <= points[i].t) {
      const a = points[i - 1];
      const b = points[i];
      const u = b.t === a.t ? 0 : (t - a.t) / (b.t - a.t);
      const s = u * u * (3 - 2 * u);
      return normalize(add(scale(a.up, 1 - s), scale(b.up, s)));
    }
  }
  return points[points.length - 1].up;
}

/**
 * Look input sweeps from -82° to +82° of elevation through the local zenith
 * and back while yaw oscillates. With allowPoleCrossing the sweep continues
 * through ±180°, passing exactly over the up vector.
 */
export function poleSweep({ frames = 720, allowPoleCrossing = true, maxPitchDeg = 178 } = {}) {
  return {
    id: allowPoleCrossing ? 'pole-cross' : 'pole-approach',
    label: allowPoleCrossing ? '穿过极点（俯仰连续跨过天顶）' : '接近极点（俯仰限制 ±89.5°）',
    frames,
    allowPoleCrossing,
    gravityUpAt: () => normalize(v3(0, 1, 0)),
    playerPosAt: null,
    lookAt: (t) => {
      const s = Math.sin(t * TAU);
      const target = allowPoleCrossing ? maxPitchDeg : 89.5;
      const pitch = target * DEG * (allowPoleCrossing ? s : Math.abs(s));
      const prevT = Math.max(0, t - 1 / (frames - 1));
      const sPrev = Math.sin(prevT * TAU);
      const prevPitch = target * DEG * (allowPoleCrossing ? sPrev : Math.abs(sPrev));
      return {
        dYaw: (26 * DEG * Math.sin(t * TAU * 2)) / frames,
        dPitch: pitch - prevPitch,
        pitch,
      };
    },
  };
}

/**
 * A slow ramp to exactly the local up axis, a dwell there while yaw turns, and
 * a ramp back down. This is the trajectory that actually enters the numerically
 * degenerate band and exercises the continuous fallback for a sustained run.
 */
export function poleDwell({ frames = 600 } = {}) {
  // Segment boundaries as fractions of the run.
  const rampIn = 0.35;
  const dwellEnd = 0.65;
  const pitchAt = (t) => {
    if (t < rampIn) return 89.4 * DEG * (t / rampIn);
    if (t < dwellEnd) return 90 * DEG;
    const u = (t - dwellEnd) / (1 - dwellEnd);
    return 90 * DEG - 89.4 * DEG * u;
  };
  return {
    id: 'pole-dwell',
    label: '极点驻留（视线正对局部上方向）',
    frames,
    allowPoleCrossing: true,
    gravityUpAt: () => normalize(v3(0, 1, 0)),
    playerPosAt: null,
    lookAt: (t) => {
      const prev = pitchAt(Math.max(0, t - 1 / (frames - 1)));
      const pitch = pitchAt(t);
      const dwelling = t >= rampIn && t < dwellEnd;
      return {
        // Yaw keeps turning while the horizon is degenerate; the frame must
        // continue rotating smoothly rather than snapping.
        dYaw: dwelling ? (12 * DEG) / frames : 0,
        dPitch: pitch - prev,
        pitch,
      };
    },
  };
}

/** Instant 180° gravity reversal (floor -> ceiling) to prove the modelled switch path. */
export function gravityFlip({ frames = 240 } = {}) {
  return {
    id: 'gravity-flip',
    label: '重力 180° 翻转',
    frames,
    discontinuous: true,
    gravityUpAt: (t) => (t < 0.5 ? v3(0, 1, 0) : v3(0, -1, 0)),
    playerPosAt: null,
    lookAt: () => ({ dYaw: 0, dPitch: 0 }),
  };
}

export function allScenarios() {
  return [
    sphereWalk(),
    wallTurn(),
    poleSweep(),
    poleSweep({ allowPoleCrossing: false, maxPitchDeg: 89.5 }),
    poleDwell(),
    gravityFlip(),
  ];
}

/**
 * Run a scenario through the real rig and return dense numeric records plus
 * aggregate statistics. This is what both the CLI and the in-page results
 * panel consume; it is the same code path as the interactive demo.
 */
export function runScenario(scenario, options = {}) {
  const rig = new GravityCameraRig({
    seedUp: scenario.gravityUpAt(0),
    allowPoleCrossing: scenario.allowPoleCrossing ?? false,
    fovDeg: options.fovDeg ?? 60,
    aspect: options.aspect ?? 16 / 9,
    limits: options.limits ?? {},
  });
  rig.setYawPitch(0, 0);
  rig.resetContinuity();
  const records = [];
  let playerPos = scenario.playerPosAt ? scenario.playerPosAt(0) : null;
  let lastGravityUp = scenario.gravityUpAt(0);

  for (let i = 0; i < scenario.frames; i += 1) {
    const t = i / (scenario.frames - 1);
    const gravityUp = scenario.gravityUpAt(t);
    if (scenario.playerPosAt) playerPos = scenario.playerPosAt(t);
    const look = scenario.lookAt(t, i);
    const prevUp = i > 0 ? scenario.gravityUpAt((i - 1) / (scenario.frames - 1)) : gravityUp;
    const jumped = scenario.discontinuous && i > 0 && dot(gravityUp, prevUp) < 0.99999;
    const pose = jumped
      ? rig.switchGravity(gravityUp)
      : rig.step({
        gravityUp,
        dYaw: look.dYaw,
        dPitch: look.dPitch,
        allowPoleCrossing: scenario.allowPoleCrossing,
      });
    lastGravityUp = gravityUp;
    // Character facing is an independent surface bearing (not the camera
    // forward), so model orientation never drives the horizon.
    const charBearing = normalize(projectTangent(pose.heading, gravityUp));
    records.push({
      i,
      t,
      gravityUp,
      playerPos,
      charFacing: charBearing,
      yawDeg: pose.yawDeg,
      pitchDeg: pose.pitchDeg,
      forward: pose.forward,
      up: pose.up,
      right: pose.right,
      quat: pose.quaternion,
      sinHorizon: pose.sinHorizon,
      solveMode: pose.solveMode,
      blendWeight: pose.blendWeight,
      orthoErrorDeg: pose.orthoErrorDeg,
      rollDeg: pose.rollDeg,
      horizonInverted: pose.horizonInverted,
      stepDeg: pose.stepDeg,
      fovOverlap: pose.fovOverlap,
      discontinuity: pose.discontinuity ?? null,
      anomalies: pose.anomalies,
    });
  }
  return { scenario: scenario.id, label: scenario.label, records, stats: summarize(records) };
}

export function summarize(records) {
  let maxOrtho = 0;
  let maxRoll = 0;
  let maxStep = 0;
  let minOverlap = 1;
  let maxRightTurn = 0;
  let maxForwardJump = 0;
  let rollSamples = 0;
  let invertedFrames = 0;
  const counts = { [SolveMode.CROSS]: 0, [SolveMode.BLEND]: 0, [SolveMode.TRANSPORT]: 0 };
  const anomalies = [];
  for (let k = 0; k < records.length; k += 1) {
    const r = records[k];
    maxOrtho = Math.max(maxOrtho, r.orthoErrorDeg);
    if (r.rollDeg != null) {
      rollSamples += 1;
      if (r.horizonInverted) {
        // View points past the zenith/nadir; the roll metric uses the nearer
        // horizon branch so it still reports genuine frame roll.
        invertedFrames += 1;
      }
      maxRoll = Math.max(maxRoll, Math.abs(r.rollDeg));
    }
    if (r.stepDeg != null) maxStep = Math.max(maxStep, r.stepDeg);
    if (r.fovOverlap != null) minOverlap = Math.min(minOverlap, r.fovOverlap);
    counts[r.solveMode] = (counts[r.solveMode] || 0) + 1;
    if (k > 0) {
      const p = records[k - 1];
      const fwdAngle = Math.acos(Math.min(1, Math.abs(
        p.forward.x * r.forward.x + p.forward.y * r.forward.y + p.forward.z * r.forward.z,
      ))) * RAD2DEG;
      const rr = Math.acos(Math.min(1, p.right.x * r.right.x + p.right.y * r.right.y + p.right.z * r.right.z)) * RAD2DEG;
      // A modelled gravity switch is an intentional re-seat; every other frame
      // must stay continuous. The switch itself is recorded separately.
      const teleport = !!(p.discontinuity || r.discontinuity);
      if (!teleport) {
        maxForwardJump = Math.max(maxForwardJump, fwdAngle);
        if (fwdAngle < 0.5) maxRightTurn = Math.max(maxRightTurn, rr);
      }
    }
    for (const a of r.anomalies) {
      if (anomalies.length < 40) anomalies.push({ i: r.i, ...a });
    }
  }
  return {
    frames: records.length,
    maxOrthoErrorDeg: maxOrtho,
    maxRollDeg: maxRoll,
    rollSamples,
    invertedFrames,
    teleports: records.filter((r) => r.discontinuity).length,
    maxStepDeg: maxStep,
    minFovOverlap: minOverlap,
    maxRightTurnWhileForwardStillDeg: maxRightTurn,
    maxForwardJumpDeg: maxForwardJump,
    modeCounts: counts,
    anomalies,
  };
}

export function evaluate(stats, limits = {}) {
  const L = {
    orthoErrorDeg: 0.05,
    rollDeg: 0.5,
    stepDeg: 25,
    fovOverlap: 0.85,
    rightTurnDeg: 12,
    ...limits,
  };
  const checks = [
    ['正交性', stats.maxOrthoErrorDeg <= L.orthoErrorDeg, `${stats.maxOrthoErrorDeg.toExponential(2)}° ≤ ${L.orthoErrorDeg}°`],
    ['地平线滚转', stats.maxRollDeg <= L.rollDeg, `${stats.maxRollDeg.toFixed(4)}° ≤ ${L.rollDeg}°`],
    ['姿态连续', stats.maxStepDeg <= L.stepDeg, `${stats.maxStepDeg.toFixed(3)}° ≤ ${L.stepDeg}°`],
    ['视野连续', stats.minFovOverlap >= L.fovOverlap, `${(stats.minFovOverlap * 100).toFixed(1)}% ≥ ${(L.fovOverlap * 100).toFixed(0)}%`],
    ['无符号翻转', stats.maxRightTurnWhileForwardStillDeg <= L.rightTurnDeg, `${stats.maxRightTurnWhileForwardStillDeg.toFixed(2)}° ≤ ${L.rightTurnDeg}°`],
    ['无数值异常', stats.anomalies.every((a) => a.level !== 'error'), `${stats.anomalies.filter((a) => a.level === 'error').length} errors`],
  ];
  return {
    pass: checks.every(([, ok]) => ok),
    checks: checks.map(([name, ok, detail]) => ({ name, ok, detail })),
  };
}
