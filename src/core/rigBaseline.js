/**
 * FROZEN GRAVITY-POSE BASELINE — do not edit.
 *
 * This is the pre-optimisation pose solver, kept verbatim as the runtime
 * reference for the pose-update optimisation. The benchmark and the parity
 * tests import this module and execute it for real; they never substitute a
 * stub. It is not imported by the application or by the scenario runner.
 */
import {
  add, angleBetween, clamp, cross, dot, len, lerp, normalize, scale, sub, v3,
} from './vec3.js';
import { quatForCamera, quatAngle } from './quat.js';

export const RAD2DEG = 180 / Math.PI;
export const DEG2RAD = Math.PI / 180;

export const SolveMode = Object.freeze({
  CROSS: 'cross',
  BLEND: 'blend',
  TRANSPORT: 'transport',
});

// |cross(forward, up)| below these values is treated as numerically degenerate.
// At 1e-4 the pitch is within 0.006° of the pole, so ordinary horizon locking
// stays active for every practical view angle (including the 89.5° pitch clamp);
// the band exists solely to hand off to the carried frame as the cross product
// loses its direction to rounding.
const HORIZON_LOW = 1e-4;
const HORIZON_HIGH = 1e-3;
const GEOM_EPS = 1e-8;
const PITCH_EPS = 0.001;
const RESEAT_DEG = 30;

export const DEFAULT_LIMITS = Object.freeze({
  orthoErrorDeg: 0.05,
  rollDeg: 0.5,
  stepDeg: 25,
  fovOverlap: 0.85,
});

const smoothstep = (a, b, x) => {
  if (a === b) return x >= b ? 1 : 0;
  const t = clamp((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
};

const wrapPi = (a) => {
  let x = (a + Math.PI) % (2 * Math.PI);
  if (x < 0) x += 2 * Math.PI;
  return x - Math.PI;
};

const projectOnPlane = (v, n) => sub(v, scale(n, dot(v, n)));

function stableTangent(up, seed) {
  const candidates = [seed, v3(0, 0, -1), v3(1, 0, 0), v3(0, 0, 1), v3(0, 1, 0)];
  for (const c of candidates) {
    if (!c) continue;
    const p = projectOnPlane(c, up);
    if (len(p) > 1e-6) return normalize(p);
  }
  return normalize(cross(up, v3(0, 1, 0)));
}

/**
 * Camera rig whose local vertical is computed from the active gravity field.
 *
 * Frame semantics:
 *   up        = normalize(-gravity)             (never a hard-coded world Y)
 *   refRight  = refForward x up                 (transported surface tangent)
 *   heading   = cos(yaw) refForward + sin(yaw) refRight
 *   forward   = cos(pitch) heading + sin(pitch) up
 *   right     = horizon-locked to `up` unless the view approaches a pole, in
 *               which case it is continuously carried from the previous frame.
 *
 * The rig never reads character animation state. Character/model facing is a
 * separate input; only the gravity up vector and look input influence the view.
 */
export class GravityCameraRig {
  constructor({
    seedUp = v3(0, 1, 0),
    seedForward = v3(0, 0, -1),
    fovDeg = 60,
    aspect = 16 / 9,
    near = 0.05,
    far = 4000,
    fps = 60,
    pitchLimitDeg = 89.5,
    allowPoleCrossing = false,
    limits = {},
  } = {}) {
    this.fovDeg = fovDeg;
    this.aspect = aspect;
    this.near = near;
    this.far = far;
    this.fps = fps;
    this.pitchLimitDeg = pitchLimitDeg;
    this.allowPoleCrossing = allowPoleCrossing;
    this.limits = { ...DEFAULT_LIMITS, ...limits };

    const U = normalize(seedUp);
    this.up = len(U) > 0.5 ? U : v3(0, 1, 0);
    this.refForward = stableTangent(this.up, seedForward);
    this.refRight = normalize(cross(this.refForward, this.up));

    this.yaw = 0;
    this.pitch = 12 * DEG2RAD;
    this.horizonParity = 1;
    this.prevPose = null;
    this.prevRight = null;
    this.history = [];
    this.historyLimit = 900;
    this.frame = 0;
    this.dominantHorizontal = v3(0, 0, -1);
    this.lastPose = this.compute();
    this.resetContinuity();
  }

  get pitchRange() {
    if (this.allowPoleCrossing) {
      return { min: -Math.PI + PITCH_EPS, max: Math.PI - PITCH_EPS };
    }
    const lim = Math.min(this.pitchLimitDeg * DEG2RAD, Math.PI / 2 - PITCH_EPS);
    return { min: -lim, max: lim };
  }

  setFov(fovDeg, aspect = this.aspect) {
    this.fovDeg = fovDeg;
    this.aspect = aspect;
  }

  setAllowPoleCrossing(flag) {
    this.allowPoleCrossing = !!flag;
    const { min, max } = this.pitchRange;
    this.pitch = clamp(this.pitch, min, max);
  }

  setYawPitch(yaw, pitch) {
    this.yaw = wrapPi(yaw);
    const { min, max } = this.pitchRange;
    this.pitch = clamp(pitch, min, max);
    return this.compute();
  }

  /**
   * Look input is defined identically in every gravity frame: yaw rotates
   * around the active gravity up, pitch raises/lowers the view toward it.
   * Dragging "right" therefore always turns the view right on screen.
   */
  addLook(dYaw, dPitch) {
    return this.setYawPitch(this.yaw + dYaw, this.pitch + dPitch);
  }

  /**
   * One simulation/render frame: apply a gravity update and a look delta, then
   * solve the pose exactly once so previous-frame metrics stay coherent.
   */
  step({ gravityUp, dYaw = 0, dPitch = 0, allowPoleCrossing } = {}) {
    if (allowPoleCrossing != null && allowPoleCrossing !== this.allowPoleCrossing) {
      this.setAllowPoleCrossing(allowPoleCrossing);
    }
    this.yaw = wrapPi(this.yaw + dYaw);
    const { min, max } = this.pitchRange;
    this.pitch = clamp(this.pitch + dPitch, min, max);
    if (gravityUp) this.applyUp(gravityUp);
    return this.compute();
  }

  /**
   * Explicit gravity re-seat for discontinuous switches (floor -> wall ->
   * ceiling). The local bearing is carried by the minimal rotation oldUp->newUp,
   * yaw/pitch are preserved, and the resulting pose carries a `discontinuity`
   * record. The frame jump is reported as a deliberate transition, and the
   * continuity baseline is restarted at the new pose so later frames measure
   * genuine per-frame continuity rather than the teleport itself.
   */
  switchGravity(newUp) {
    const U = normalize(newUp);
    if (!(len(U) > 0.5)) return this.compute();
    const oldUp = this.up;
    const deltaDeg = angleBetween(oldUp, U) * RAD2DEG;
    this.applyUp(U);
    // A modelled switch is a deliberate re-seat: do not carry the old frame's
    // sign branch, because for a gravity reversal the correct local horizon is
    // rotated 180° in world space. Build the horizon from the new gravity only.
    this.prevRight = null;
    this.prevPose = null;
    const pose = this.compute();
    const discontinuity = {
      type: 'GRAVITY_SWITCH',
      fromUp: oldUp,
      toUp: U,
      angleDeg: deltaDeg,
      frame: pose.frame,
      note: deltaDeg > 90
        ? '重力反转：世界空间姿态跳变 180°，局部上方向已按新重力重建'
        : '重力切换：相机姿态按新局部上方向重定向',
    };
    pose.discontinuity = discontinuity;
    pose.anomalies = pose.anomalies.filter((a) => a.code !== 'STEP' && a.code !== 'FOV');
    if (deltaDeg > 90) {
      pose.anomalies.push({ level: 'info', code: 'GRAVITY_REVERSE', message: discontinuity.note });
    }
    this.prevPose = pose;
    this.prevRight = pose.right;
    return pose;
  }

  /**
   * Update the active gravity up vector. The tangent reference is carried along
   * the minimal rotation between the old and new up vectors, so yaw=0 keeps
   * pointing along the same surface bearing instead of snapping to world north.
   */
  setUp(newUp) {
    this.applyUp(newUp);
    return this.compute();
  }

  /**
   * Set gravity, tangent reference, and look angles together, then solve once.
   * Used for mode entry and teleports. Seeding the carried horizon from the
   * supplied reference keeps the frame upright and roll-free at the pole's
   * opposite branch instead of inheriting a stale frame.
   */
  configure({ gravityUp, referenceForward = null, yaw = 0, pitch = 12 * DEG2RAD } = {}) {
    if (gravityUp) {
      const U = normalize(gravityUp);
      if (len(U) > 0.5) this.up = U;
    }
    const base = referenceForward || this.refForward;
    const tangent = projectOnPlane(base, this.up);
    this.refForward = len(tangent) > 1e-7 ? normalize(tangent) : stableTangent(this.up, base);
    this.refRight = normalize(cross(this.refForward, this.up));
    this.yaw = wrapPi(yaw);
    const { min, max } = this.pitchRange;
    this.pitch = clamp(pitch, min, max);
    this.horizonParity = 1;
    this.prevPose = null;
    this.prevRight = null;
    const pose = this.compute();
    this.prevPose = pose;
    this.prevRight = pose.right;
    return pose;
  }

  applyUp(newUp) {
    const U = normalize(newUp);
    if (!(len(U) > 0.5)) return;
    const c = clamp(dot(this.up, U), -1, 1);
    if (c < 1 - 1e-13) {
      const axis = cross(this.up, U);
      const s = len(axis);
      let carried = this.refForward;
      if (s > 1e-10) {
        const angle = Math.atan2(s, c);
        // Rodrigues rotation of the tangent bearing by the minimal up rotation.
        const ca = Math.cos(angle);
        const sa = Math.sin(angle);
        const k = scale(axis, 1 / s);
        carried = add(
          add(scale(this.refForward, ca), scale(cross(k, this.refForward), sa)),
          scale(k, dot(k, this.refForward) * (1 - ca)),
        );
      }
      // For an exact antiparallel flip (s ~ 0, c < 0) a 180° rotation about
      // refForward is chosen: the bearing is preserved and the result is
      // deterministic instead of depending on a noisy cross product axis.
      const tangent = projectOnPlane(carried, U);
      if (len(tangent) > 1e-7) this.refForward = normalize(tangent);
      else this.refForward = stableTangent(U, this.refForward);
    }
    this.up = U;
    this.refForward = normalize(projectOnPlane(this.refForward, this.up));
    this.refRight = normalize(cross(this.refForward, this.up));
  }

  setReferenceForward(direction) {
    const tangent = projectOnPlane(direction, this.up);
    if (len(tangent) > 1e-7) {
      this.refForward = normalize(tangent);
      this.refRight = normalize(cross(this.refForward, this.up));
    }
    return this.compute();
  }

  resetLook() {
    this.yaw = 0;
    this.pitch = 12 * DEG2RAD;
    return this.compute();
  }

  /** Drop the previous-frame baseline after teleporting the rig. */
  resetContinuity() {
    this.prevPose = null;
    this.prevRight = this.lastPose ? this.lastPose.right : this.prevRight;
  }

  solveHorizon(forward) {
    const U = this.up;
    const raw = cross(forward, U);
    const s = len(raw);

    // Only build a normalized cross axis where the cross product is numerically
    // well-conditioned. Below HORIZON_LOW it is discarded entirely; a near-zero
    // vector is never normalized as the sole source of the horizon.
    const natural = s > HORIZON_LOW ? scale(raw, 1 / s) : null;

    let transported = null;
    if (this.prevRight) {
      const p = projectOnPlane(this.prevRight, forward);
      if (len(p) > 1e-8) transported = normalize(p);
    }

    // Reconcile the cross-product branch with the carried frame every frame.
    // The sign of cross(forward, up) flips whenever the view passes either
    // pole, including between samples; comparing against the carried axis
    // turns that flip into a continuous rotation instead of a 180° snap.
    if (natural && transported) {
      this.horizonParity = dot(natural, transported) < 0 ? -1 : 1;
    }

    let right = null;
    let mode;
    let weight = 1;

    if (s >= HORIZON_HIGH && natural) {
      right = scale(natural, this.horizonParity);
      mode = SolveMode.CROSS;
    } else if (natural && transported) {
      // Inside the degenerate band. The gravity horizon has no usable sign
      // here, so the carried frame supplies the direction and the cross axis
      // fades in continuously.
      weight = smoothstep(HORIZON_LOW, HORIZON_HIGH, s);
      const cand = scale(natural, this.horizonParity);
      const mixed = lerp(transported, cand, weight);
      right = len(mixed) > 1e-7 ? normalize(mixed) : transported;
      mode = SolveMode.BLEND;
    } else if (transported) {
      // Fully degenerate (|f x u| < HORIZON_LOW): the carried tangent frame
      // alone defines the horizon.
      right = transported;
      mode = SolveMode.TRANSPORT;
      weight = 0;
    } else {
      // No carried frame yet (first frame, or after a teleport): seed the
      // horizon from the gravity tangent reference.
      const fb = projectOnPlane(this.refRight, forward);
      right = len(fb) > 1e-7 ? normalize(fb) : stableTangent(forward, this.refForward);
      mode = SolveMode.TRANSPORT;
      weight = 0;
      if (natural) this.horizonParity = dot(natural, right) < 0 ? -1 : 1;
    }

    right = projectOnPlane(right, forward);
    if (len(right) < 1e-8) right = stableTangent(forward, this.refForward);
    right = normalize(right);
    let camUp = cross(right, forward);
    if (len(camUp) < 1e-8) camUp = stableTangent(forward, U);
    camUp = normalize(camUp);

    return { right, camUp, sinHorizon: s, mode, weight, parity: this.horizonParity };
  }

  compute() {
    const U = this.up;
    const cy = Math.cos(this.yaw);
    const sy = Math.sin(this.yaw);
    const heading = normalize(add(scale(this.refForward, cy), scale(this.refRight, sy)));
    const cp = Math.cos(this.pitch);
    const sp = Math.sin(this.pitch);
    // At a pitch of exactly ±90° the horizontal heading has no influence on
    // the view; build forward from the surface basis directly so a zeroed
    // heading cannot collapse the direction vector.
    const forward = normalize(add(
      add(scale(this.refForward, cp * cy), scale(this.refRight, cp * sy)),
      scale(U, sp),
    ));

    const horizon = this.solveHorizon(forward);
    const { right, camUp } = horizon;

    const quat = quatForCamera(right, camUp, forward);

    const orthoErrors = [
      Math.abs(dot(right, camUp)),
      Math.abs(dot(camUp, forward)),
      Math.abs(dot(forward, right)),
      Math.abs(len(right) - 1),
      Math.abs(len(camUp) - 1),
      Math.abs(len(forward) - 1),
    ];
    const orthoErrorDeg = Math.max(...orthoErrors) * RAD2DEG;
    const horiz = projectOnPlane(forward, U);
    if (len(horiz) > 1e-4) this.dominantHorizontal = normalize(horiz);

    // Roll is the signed angle between the camera up axis and the gravity
    // horizon's up direction, measured about the view axis. The horizon has two
    // sign branches: when the view points past the zenith/nadir its projection
    // is inverted, so the physically meaningful comparison is against the
    // nearer branch. This keeps |roll| <= 90° and reports genuine frame roll,
    // not the unavoidable sign of the projection.
    const horizonUp = projectOnPlane(U, forward);
    let rollDeg = null;
    let horizonInverted = false;
    if (len(horizonUp) > 1e-4) {
      const hUp = normalize(horizonUp);
      let sinR = dot(cross(hUp, camUp), forward);
      let cosR = dot(hUp, camUp);
      // Choose the branch that yields the smaller rotation from camUp.
      if (cosR < 0) {
        sinR = -sinR;
        cosR = -cosR;
        horizonInverted = true;
      }
      rollDeg = Math.atan2(sinR, cosR) * RAD2DEG;
    }

    const halfV = this.fovDeg * DEG2RAD * 0.5;
    const halfH = Math.atan(Math.tan(halfV) * this.aspect);
    const halfDiag = Math.atan(Math.hypot(Math.tan(halfV), Math.tan(halfH)));

    const pose = {
      frame: this.frame,
      time: this.frame / this.fps,
      gravityUp: U,
      gravity: scale(U, -1),
      referenceForward: this.refForward,
      referenceRight: this.refRight,
      yaw: this.yaw,
      yawDeg: this.yaw * RAD2DEG,
      pitch: this.pitch,
      pitchDeg: this.pitch * RAD2DEG,
      heading,
      forward,
      up: camUp,
      right,
      back: scale(forward, -1),
      quaternion: quat,
      fovDeg: this.fovDeg,
      fovHorizontalDeg: halfH * 2 * RAD2DEG,
      fovHalfDiagDeg: halfDiag * RAD2DEG,
      aspect: this.aspect,
      sinHorizon: horizon.sinHorizon,
      solveMode: horizon.mode,
      blendWeight: horizon.weight,
      orthoErrorDeg,
      rollDeg,
      horizonInverted,
      stepDeg: null,
      forwardStepDeg: null,
      upStepDeg: null,
      rightStepDeg: null,
      quatStepDeg: null,
      fovOverlap: null,
      anomalies: [],
    };

    if (this.prevPose) {
      pose.forwardStepDeg = angleBetween(this.prevPose.forward, forward) * RAD2DEG;
      pose.upStepDeg = angleBetween(this.prevPose.up, camUp) * RAD2DEG;
      pose.rightStepDeg = angleBetween(this.prevPose.right, right) * RAD2DEG;
      pose.quatStepDeg = quatAngle(this.prevPose.quaternion, quat) * RAD2DEG;
      pose.stepDeg = Math.max(pose.forwardStepDeg, pose.upStepDeg, pose.rightStepDeg, pose.quatStepDeg);
      pose.fovOverlap = frustumOverlap(this.prevPose, pose, this.fovDeg, this.aspect);
      pose.anomalies = this.detectAnomalies(pose);
    }

    this.prevPose = pose;
    this.prevRight = right;
    this.lastPose = pose;
    this.frame += 1;
    this.history.push({
      frame: pose.frame,
      time: pose.time,
      yawDeg: pose.yawDeg,
      pitchDeg: pose.pitchDeg,
      up: U,
      forward,
      quat,
      sinHorizon: pose.sinHorizon,
      solveMode: pose.solveMode,
      blendWeight: pose.blendWeight,
      orthoErrorDeg,
      rollDeg,
      stepDeg: pose.stepDeg,
      fovOverlap: pose.fovOverlap,
      anomalies: pose.anomalies,
    });
    if (this.history.length > this.historyLimit) this.history.shift();
    return pose;
  }

  detectAnomalies(pose) {
    const out = [];
    const vals = [
      ...Object.values(pose.forward), ...Object.values(pose.up), ...Object.values(pose.right),
      ...Object.values(pose.quaternion), ...Object.values(pose.gravityUp),
    ];
    if (vals.some((v) => !Number.isFinite(v))) {
      out.push({ level: 'error', code: 'NON_FINITE', message: '相机姿态出现非有限数值' });
      return out;
    }
    if (pose.solveMode !== SolveMode.CROSS) {
      out.push({
        level: 'info',
        code: 'POLE_FALLBACK',
        message: pose.solveMode === SolveMode.TRANSPORT
          ? '极点回退：沿上一帧切向连续求解'
          : '接近极点：叉乘与沿移按权重连续混合',
      });
    }
    if (Math.abs(pose.rollDeg ?? 0) > this.limits.rollDeg) {
      out.push({ level: 'warn', code: 'ROLL', message: `相机滚转 ${pose.rollDeg.toFixed(3)}° 超出阈值` });
    }
    if (pose.orthoErrorDeg > this.limits.orthoErrorDeg) {
      out.push({ level: 'warn', code: 'ORTHO', message: `基向量正交误差 ${pose.orthoErrorDeg.toExponential(2)} 度` });
    }
    if (pose.stepDeg != null && pose.stepDeg > this.limits.stepDeg) {
      out.push({ level: 'warn', code: 'STEP', message: `单帧姿态跳变 ${pose.stepDeg.toFixed(2)}°` });
    }
    if (pose.fovOverlap != null && pose.fovOverlap < this.limits.fovOverlap) {
      out.push({
        level: pose.fovOverlap < 0.5 ? 'error' : 'warn',
        code: 'FOV',
        message: `视锥重叠仅 ${(pose.fovOverlap * 100).toFixed(1)}%`,
      });
    }
    return out;
  }
}

/**
 * Fraction of a 5x5 ray grid of the previous frustum that still lies inside the
 * current frustum. Uses the actual camera bases and FOV aspect, not an
 * approximation based only on the forward-axis angle.
 */
export function frustumOverlap(prev, curr, fovDeg, aspect, samples = 7, inset = 0.9) {
  const tv = Math.tan(fovDeg * DEG2RAD * 0.5);
  const th = tv * aspect;
  let inside = 0;
  let total = 0;
  for (let i = 0; i < samples; i += 1) {
    const u = (-1 + (2 * i) / (samples - 1)) * inset;
    for (let j = 0; j < samples; j += 1) {
      const v = (-1 + (2 * j) / (samples - 1)) * inset;
      const dir = normalize(add(
        add(prev.forward, scale(prev.right, u * th)),
        scale(prev.up, v * tv),
      ));
      const z = dot(dir, curr.forward);
      total += 1;
      if (z <= 1e-6) continue;
      const x = dot(dir, curr.right) / z;
      const y = dot(dir, curr.up) / z;
      if (Math.abs(x) <= th + 1e-9 && Math.abs(y) <= tv + 1e-9) inside += 1;
    }
  }
  return inside / total;
}

export function cameraPositionFromPivot(pose, pivot, { boom = 6, height = 1.2, surfaceUp = null, minClearance = 0 } = {}) {
  const U = surfaceUp || pose.gravityUp;
  const horizontal = projectOnPlane(pose.forward, U);
  const back = len(horizontal) > 1e-5 ? normalize(horizontal) : pose.referenceForward;
  let position = add(add(pivot, scale(U, height)), scale(back, -boom));
  if (minClearance > 0) {
    const h = dot(position, U);
    const floor = dot(pivot, U) + minClearance;
    if (h < floor) position = add(position, scale(U, floor - h));
  }
  return position;
}
