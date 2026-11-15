import * as THREE from 'three';
import {
  GravityCameraRig, SolveMode, RAD2DEG, DEG2RAD, cameraPositionFromPivot,
} from './core/rig.js';
import {
  add, cross, dot, len, normalize, scale, sub, v3,
} from './core/vec3.js';
import {
  buildRenderer, buildWorld, CHAMBER_HALF, CHAMBER_HEIGHT, PLANET_RADIUS, PLAYER_HEIGHT,
} from './demo/world.js';
import {
  allScenarios, runScenario, evaluate, gravityPresets,
} from './sim/scenarios.js';

const $ = (id) => document.getElementById(id);

const state = {
  mode: 'planet',
  gravityId: 'G_FLOOR',
  gravityUp: v3(0, 1, 0),
  yaw: 0,
  pitch: 12 * DEG2RAD,
  characterYaw: 0,
  boom: 5.5,
  sens: 1,
  fov: 60,
  allowPoleCrossing: false,
  autoTurntable: false,
  playerPos: null,
  playing: false,
  scenarioRun: null,
  scenarioProgress: 0,
  scenarioClock: 0,
  dragging: false,
  lastPointer: { x: 0, y: 0 },
  walkPhase: 0,
  samples: [],
  lastAnomalyKeys: new Set(),
  ghost: false,
  bank: 0,
};

const canvas = $('scene');
const renderer = buildRenderer(canvas);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.05, 4000);
const world = buildWorld(scene);
const rig = new GravityCameraRig({
  seedUp: v3(0, 1, 0),
  seedForward: v3(0, 0, -1),
  fovDeg: 60,
  aspect: 16 / 9,
});

// ---------------------------------------------------------------------------
// Trail and frustum visualisers
// ---------------------------------------------------------------------------
const history = {
  fwd: [],
  up: [],
  right: [],
  limit: 600,
};

const ghostGroup = new THREE.Group();
scene.add(ghostGroup);
const ghostMatPrev = new THREE.LineBasicMaterial({ color: 0x5fe0a4, transparent: true, opacity: 0.55 });
const ghostMatCurr = new THREE.LineBasicMaterial({ color: 0xffd479, transparent: true, opacity: 0.85 });
let prevGhost = null;
let currGhost = null;

function frustumLines(camPos, forward, up, right, fovDeg, aspect, near, far) {
  const tv = Math.tan((fovDeg * DEG2RAD) / 2);
  const th = tv * aspect;
  const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
  const farPts = corners.map(([u, w]) => add(
    add(add(camPos, scale(forward, far)), scale(right, u * th * far)),
    scale(up, w * tv * far),
  ));
  const nearPts = corners.map(([u, w]) => add(
    add(add(camPos, scale(forward, near)), scale(right, u * th * near)),
    scale(up, w * tv * near),
  ));
  const pts = [];
  for (let i = 0; i < 4; i += 1) {
    pts.push(nearPts[i], farPts[i]);
    pts.push(farPts[i], farPts[(i + 1) % 4]);
    pts.push(nearPts[i], nearPts[(i + 1) % 4]);
  }
  // Rays from viewpoint.
  for (const p of farPts) pts.push(camPos, p);
  return pts.map((p) => new THREE.Vector3(p.x, p.y, p.z));
}

function updateGhosts(pos, pose) {
  if (!state.ghost) {
    ghostGroup.visible = false;
    return;
  }
  ghostGroup.visible = true;
  if (prevGhost) ghostGroup.remove(prevGhost);
  if (currGhost) ghostGroup.remove(currGhost);
  if (rig.prevPose) {
    prevGhost = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(
        frustumLines(pos, rig.prevPose.forward, rig.prevPose.up, rig.prevPose.right, rig.fovDeg, rig.aspect, 1, 8),
      ),
      ghostMatPrev,
    );
    ghostGroup.add(prevGhost);
  }
  currGhost = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(
      frustumLines(pos, pose.forward, pose.up, pose.right, rig.fovDeg, rig.aspect, 1, 8),
    ),
    ghostMatCurr,
  );
  ghostGroup.add(currGhost);
}

// ---------------------------------------------------------------------------
// Definition of gravity surfaces for each mode
// ---------------------------------------------------------------------------
function chamberSurfaces() {
  return [
    { id: 'G_FLOOR', label: '地面 +Y', up: v3(0, 1, 0), stand: v3(0, 0, 0) },
    { id: 'G_CEILING', label: '天花板 -Y', up: v3(0, -1, 0), stand: v3(0, CHAMBER_HEIGHT, 0) },
    { id: 'G_WEST', label: '西墙 +X', up: v3(1, 0, 0), stand: v3(-CHAMBER_HALF, 4, 0) },
    { id: 'G_EAST', label: '东墙 -X', up: v3(-1, 0, 0), stand: v3(CHAMBER_HALF, 4, 0) },
    { id: 'G_SLOPE', label: '斜面 35°', up: normalize(v3(-Math.sin(35 * DEG2RAD), Math.cos(35 * DEG2RAD), 0)), stand: v3(-14, 7.6, 12) },
    { id: 'G_TILT', label: '斜角重力', up: normalize(v3(0.5, 0.85, 0.2)), stand: v3(8, 6, -6) },
  ];
}

function applyMode(mode) {
  state.mode = mode;
  document.querySelectorAll('.mode-btn').forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $('grp-chamber').style.display = mode === 'chamber' ? '' : 'none';
  $('grp-scenario').style.display = mode === 'scenario' ? '' : 'none';
  $('grp-legend').style.display = mode === 'scenario' ? 'none' : '';
  $('scene-state').textContent = {
    planet: 'planet · 重力 = 径向',
    chamber: 'chamber · 表面法线',
    scenario: 'scenario · 轨迹回放',
  }[mode];

  if (mode === 'planet') {
    state.gravityId = 'PLANET';
    const dir = normalize(v3(0.28, 0.18, 0.94));
    state.playerPos = scale(dir, PLANET_RADIUS + PLAYER_HEIGHT);
    state.gravityUp = dir;
    rig.configure({
      gravityUp: dir,
      referenceForward: normalize(cross(dir, v3(0, 0, 1))),
      yaw: 0,
      pitch: 8 * DEG2RAD,
    });
    world.planet.visible = true;
    world.chamber.group.visible = false;
  } else if (mode === 'chamber' || mode === 'scenario') {
    if (mode === 'chamber') {
      const surfaces = chamberSurfaces();
      const chosen = surfaces.find((s) => s.id === state.gravityId) || surfaces[0];
      state.playerPos = add(chosen.stand, scale(chosen.up, PLAYER_HEIGHT));
      state.gravityUp = chosen.up;
      rig.configure({
        gravityUp: chosen.up,
        referenceForward: normalize(cross(chosen.up, v3(0, 0, 1))),
        yaw: 0,
        pitch: 8 * DEG2RAD,
      });
    }
    world.planet.visible = false;
    world.chamber.group.visible = true;
  }
  if (mode === 'scenario') {
    stopScenario();
    state.playerPos = add(v3(0, 0, 0), scale(v3(0, 1, 0), PLAYER_HEIGHT));
    world.planet.visible = false;
    world.chamber.group.visible = true;
  }
  syncGravityButtons();
  refreshSummary();
}

function setGravityUp(up, { reseat = false, pose = false } = {}) {
  state.gravityUp = normalize(up);
  if (reseat) {
    rig.switchGravity(state.gravityUp);
  } else {
    rig.setUp(state.gravityUp);
  }
  if (pose) rig.setYawPitch(rig.yaw, rig.pitch);
}

// ---------------------------------------------------------------------------
// Character: facing and up are separate from camera horizon control
// ---------------------------------------------------------------------------
function setCharacterYaw(yaw) {
  state.characterYaw = yaw;
  state.characterYawVec = normalize(add(
    scale(rig.refForward, Math.cos(yaw)),
    scale(rig.refRight, Math.sin(yaw)),
  ));
}

function updateCharacter(dt) {
  const p = state.playerPos;
  const group = world.player.group;
  const up = state.gravityUp;
  // playerPos is the body centre; the model's origin is at its feet, so the
  // offset must be along the active gravity up, not world Y.
  const feet = sub(p, scale(up, PLAYER_HEIGHT));
  group.position.set(feet.x, feet.y, feet.z);
  if (!state.characterYawVec) setCharacterYaw(0);
  const fwd = normalize(sub(state.characterYawVec, scale(up, dot(state.characterYawVec, up))));
  const right = normalize(cross(fwd, up));
  const m = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(right.x, right.y, right.z),
    new THREE.Vector3(up.x, up.y, up.z),
    new THREE.Vector3(-fwd.x, -fwd.y, -fwd.z),
  );
  group.quaternion.setFromRotationMatrix(m);

  // Model-only animation: a small bob and body yaw wobble. This must never
  // reach the camera, which is why the rig never reads these values.
  state.walkPhase += dt * (state.moving ? 9 : 0);
  const bob = state.moving ? Math.abs(Math.sin(state.walkPhase)) * 0.07 : 0;
  const sway = state.moving ? Math.sin(state.walkPhase * 0.5) * 0.06 : Math.sin(performance.now() * 0.0011) * 0.015;
  world.player.parts.body.position.y = 0.95 + bob;
  world.player.parts.head.position.y = 1.66 + bob;
  world.player.parts.body.rotation.z = sway;
  world.player.parts.bootL.position.z = -0.03 + Math.sin(state.walkPhase) * (state.moving ? 0.18 : 0);
  world.player.parts.bootR.position.z = -0.03 - Math.sin(state.walkPhase) * (state.moving ? 0.18 : 0);

  $('char-facing').textContent = state.characterYawVec
    ? `(${state.characterYawVec.x.toFixed(2)}, ${state.characterYawVec.y.toFixed(2)}, ${state.characterYawVec.z.toFixed(2)})`
    : '—';
  $('char-up').textContent = `(${up.x.toFixed(2)}, ${up.y.toFixed(2)}, ${up.z.toFixed(2)})`;
}

// ---------------------------------------------------------------------------
// Movement on arbitrary surfaces
// ---------------------------------------------------------------------------
function moveOnSurface(localForward, localRight, dt) {
  const up = state.gravityUp;
  const speed = 9 * dt;
  const delta = add(scale(localForward, speed * state.input.f), scale(localRight, speed * state.input.r));
  const tangential = sub(delta, scale(up, dot(delta, up)));
  state.playerPos = add(state.playerPos, tangential);
  state.moving = len(tangential) > 1e-6;
  if (state.moving) state.characterYawVec = normalize(tangential);

  if (state.mode === 'planet') {
    const dir = normalize(sub(state.playerPos, v3(0, 0, 0)));
    state.playerPos = scale(dir, PLANET_RADIUS + PLAYER_HEIGHT);
    // The tangent bearing is carried by the rig's own minimal-rotation update,
    // so no per-frame reference reset (which would snap at the world poles).
    state.characterYawVec = normalize(sub(state.characterYawVec, scale(dir, dot(state.characterYawVec, dir))));
  } else {
    clampToChamber();
  }
  if (state.mode === 'chamber') {
    const surface = chamberSurfaces().find((s) => angleClose(s.up, up));
    if (surface) {
      // Keep the player glued to the chosen surface.
      const h = dot(state.playerPos, surface.up);
      const targetH = dot(surface.stand, surface.up) + PLAYER_HEIGHT;
      state.playerPos = add(state.playerPos, scale(surface.up, targetH - h));
    }
  }
}

function angleClose(a, b) {
  return dot(normalize(a), normalize(b)) > 0.9995;
}

function clampToChamber() {
  const lim = CHAMBER_HALF - 1.2;
  const p = state.playerPos;
  const clamped = v3(
    Math.max(-lim, Math.min(lim, p.x)),
    Math.max(0.2, Math.min(CHAMBER_HEIGHT - 0.2, p.y)),
    Math.max(-lim, Math.min(lim, p.z)),
  );
  state.playerPos = clamped;
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------
state.input = { f: 0, r: 0 };
const keys = new Set();

window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
  keys.add(e.code);
  if (e.code === 'Space') {
    state.allowPoleCrossing = !state.allowPoleCrossing;
    $('pole-toggle').checked = state.allowPoleCrossing;
    rig.setAllowPoleCrossing(state.allowPoleCrossing);
    toastAnomaly({ level: 'info', code: 'POLE_MODE', message: state.allowPoleCrossing ? '已允许穿过极点：使用连续回退' : '已禁止穿过极点：俯仰限制在 ±89.5°' });
  }
  if (e.code === 'KeyQ') setCharacterYaw(state.characterYaw - 15 * DEG2RAD);
  if (e.code === 'KeyE') setCharacterYaw(state.characterYaw + 15 * DEG2RAD);
});
window.addEventListener('keyup', (e) => keys.delete(e.code));

canvas.addEventListener('pointerdown', (e) => {
  state.dragging = true;
  state.lastPointer = { x: e.clientX, y: e.clientY };
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointerup', (e) => {
  state.dragging = false;
  try { canvas.releasePointerCapture(e.pointerId); } catch { /* ignore */ }
});
canvas.addEventListener('pointermove', (e) => {
  if (!state.dragging) return;
  const dx = e.clientX - state.lastPointer.x;
  const dy = e.clientY - state.lastPointer.y;
  state.lastPointer = { x: e.clientX, y: e.clientY };
  const k = 0.0042 * state.sens;
  rig.setYawPitch(rig.yaw - dx * k, rig.pitch - dy * k);
});
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  state.boom = Math.max(2.5, Math.min(12, state.boom + Math.sign(e.deltaY) * 0.45));
  $('boom-range').value = String(state.boom);
  $('boom-val').textContent = `${state.boom.toFixed(1)}m`;
}, { passive: false });

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------
const gravityButtons = $('gravity-buttons');
for (const s of chamberSurfaces()) {
  const btn = document.createElement('button');
  btn.dataset.id = s.id;
  btn.textContent = s.label;
  btn.addEventListener('click', () => {
    state.gravityId = s.id;
    state.playerPos = add(s.stand, scale(s.up, PLAYER_HEIGHT));
    state.gravityUp = s.up;
    rig.configure({
      gravityUp: s.up,
      referenceForward: normalize(cross(s.up, v3(0, 0, 1))),
      yaw: 0,
      pitch: 8 * DEG2RAD,
    });
    syncGravityButtons();
    $('gravity-hint').textContent = `当前局部上方向 = (${s.up.x.toFixed(2)}, ${s.up.y.toFixed(2)}, ${s.up.z.toFixed(2)})`;
    toastAnomaly({
      level: 'info',
      code: 'GRAVITY_SWITCH',
      message: `重力切换至 ${s.label}；相机上方向由该表面法线计算`,
    });
  });
  gravityButtons.appendChild(btn);
}

function syncGravityButtons() {
  let active = state.gravityId;
  document.querySelectorAll('#gravity-buttons button').forEach((b) => b.classList.toggle('active', b.dataset.id === active));
}

document.querySelectorAll('.mode-btn').forEach((b) => {
  b.addEventListener('click', () => applyMode(b.dataset.mode));
});

$('fov-range').addEventListener('input', (e) => {
  state.fov = Number(e.target.value);
  $('fov-val').textContent = `${state.fov}°`;
  rig.setFov(state.fov, camera.aspect);
});
$('sens-range').addEventListener('input', (e) => {
  state.sens = Number(e.target.value);
  $('sens-val').textContent = `${state.sens.toFixed(1)}×`;
});
$('boom-range').addEventListener('input', (e) => {
  state.boom = Number(e.target.value);
  $('boom-val').textContent = `${state.boom.toFixed(1)}m`;
});
$('pole-toggle').addEventListener('change', (e) => {
  state.allowPoleCrossing = e.target.checked;
  rig.setAllowPoleCrossing(state.allowPoleCrossing);
});
$('ghost-toggle').addEventListener('change', (e) => {
  state.ghost = e.target.checked;
  if (!state.ghost) ghostGroup.visible = false;
});
$('btn-face-left').addEventListener('click', () => setCharacterYaw(state.characterYaw - 15 * DEG2RAD));
$('btn-face-right').addEventListener('click', () => setCharacterYaw(state.characterYaw + 15 * DEG2RAD));
$('btn-export').addEventListener('click', exportSamples);
$('btn-reset').addEventListener('click', () => {
  state.samples = [];
  $('sample-count').textContent = '0';
  rig.history.length = 0;
});

// Scenario controls
const scenarioSelect = $('scenario-select');
const scenarios = allScenarios();
for (const s of scenarios) {
  const opt = document.createElement('option');
  opt.value = s.id;
  opt.textContent = s.label;
  scenarioSelect.appendChild(opt);
}
scenarioSelect.addEventListener('change', () => {
  state.scenarioRun = null;
  state.scenarioProgress = 0;
  $('scenario-progress').value = '0';
  $('summary').innerHTML = '选择“播放”运行并记录数值。';
});

function stopScenario() {
  state.playing = false;
  $('btn-play').textContent = '播放';
}

$('btn-play').addEventListener('click', () => {
  state.playing = !state.playing;
  $('btn-play').textContent = state.playing ? '暂停' : '播放';
  if (state.playing && !state.scenarioRun) prepareScenario();
  if (!state.playing) $('btn-play').textContent = '播放';
});

$('btn-restart').addEventListener('click', () => {
  state.scenarioRun = null;
  state.scenarioProgress = 0;
  prepareScenario();
  state.playing = true;
  $('btn-play').textContent = '暂停';
});

function prepareScenario() {
  const s = scenarios.find((x) => x.id === scenarioSelect.value) || scenarios[0];
  const run = runScenario(s, { fovDeg: state.fov, aspect: camera.aspect });
  state.scenarioRun = run;
  state.scenarioProgress = 0;
  state.scenarioClock = 0;
  state.scenarioViewPos = null;
  rig.setAllowPoleCrossing(!!s.allowPoleCrossing);
  rig.setYawPitch(0, 0);
  rig.resetContinuity();
  state.playerPos = add(v3(0, 0, 0), scale(v3(0, 1, 0), PLAYER_HEIGHT));
  state.gravityUp = v3(0, 1, 0);
  rig.setUp(v3(0, 1, 0));
  state.characterYawVec = normalize(v3(0, 0, -1));
  $('scenario-state').textContent = `帧 0/${run.records.length} · 准备播放`;
  refreshSummary();
}

function refreshSummary() {
  const run = state.scenarioRun;
  const el = $('summary');
  if (!run) {
    if (state.mode === 'scenario') el.innerHTML = '选择“播放”运行并记录数值。';
    else el.innerHTML = '选择“输入轨迹”并播放后显示。';
    return;
  }
  const ev = evaluate(run.stats);
  const modeTotal = run.stats.frames;
  const rows = ev.checks.map((c) => `
    <div class="check">
      <span>${c.name} <span class="detail">${c.detail}</span></span>
      <b class="${c.ok ? 'ok' : 'no'}">${c.ok ? '通过' : '未通过'}</b>
    </div>`).join('');
  el.innerHTML = `
    <div class="head"><span>${run.label}</span><span class="${ev.pass ? 'ok' : 'no'}">${ev.pass ? '全部通过' : '存在异常'}</span></div>
    ${rows}
    <div class="check"><span>解算模式分布</span><b class="detail">cross ${run.stats.modeCounts.cross} · blend ${run.stats.modeCounts.blend} · transport ${run.stats.modeCounts.transport}</b></div>
    <div class="check"><span>帧数</span><b class="detail">${modeTotal}</b></div>`;
}

// ---------------------------------------------------------------------------
// Results panels
// ---------------------------------------------------------------------------
const chartCanvas = $('chart');
const chartCtx = chartCanvas.getContext('2d');
const lensCanvas = $('lens');
const lensCtx = lensCanvas.getContext('2d');
const metricHistory = { roll: [], step: [], overlap: [] };

function fmtVec(v) {
  return `(${v.x.toFixed(2)}, ${v.y.toFixed(2)}, ${v.z.toFixed(2)})`;
}

function updateReadouts(pose) {
  $('v-forward').textContent = fmtVec(pose.forward);
  $('v-up').textContent = fmtVec(pose.up);
  $('v-right').textContent = fmtVec(pose.right);
  $('v-quat').textContent = `(${pose.quaternion.x.toFixed(2)}, ${pose.quaternion.y.toFixed(2)}, ${pose.quaternion.z.toFixed(2)}, ${pose.quaternion.w.toFixed(2)})`;
  $('v-yaw').textContent = `${pose.yawDeg.toFixed(1)}°`;
  $('v-pitch').textContent = `${pose.pitchDeg.toFixed(1)}°`;
  $('v-gup').textContent = fmtVec(pose.gravityUp);
  $('v-mode').textContent = {
    [SolveMode.CROSS]: '叉乘地平线锁',
    [SolveMode.BLEND]: '连续混合回退',
    [SolveMode.TRANSPORT]: '沿移回退',
  }[pose.solveMode] || pose.solveMode;

  const ortho = $('m-ortho');
  ortho.textContent = pose.orthoErrorDeg.toExponential(2);
  ortho.className = pose.orthoErrorDeg < 0.05 ? 'good' : pose.orthoErrorDeg < 0.5 ? 'warn' : 'bad';

  const roll = $('m-roll');
  roll.textContent = pose.rollDeg == null ? '极点·无定义' : `${pose.rollDeg.toFixed(3)}°`;
  roll.className = pose.rollDeg == null ? 'warn' : Math.abs(pose.rollDeg) < 0.5 ? 'good' : Math.abs(pose.rollDeg) < 5 ? 'warn' : 'bad';

  const step = $('m-step');
  step.textContent = pose.stepDeg == null ? '—' : `${pose.stepDeg.toFixed(3)}°`;
  step.className = pose.stepDeg == null ? '' : pose.stepDeg < 5 ? 'good' : pose.stepDeg < 25 ? 'warn' : 'bad';

  const ov = $('m-overlap');
  ov.textContent = pose.fovOverlap == null ? '—' : `${(pose.fovOverlap * 100).toFixed(1)}%`;
  ov.className = pose.fovOverlap == null ? '' : pose.fovOverlap > 0.95 ? 'good' : pose.fovOverlap > 0.85 ? 'warn' : 'bad';

  $('m-sinhorizon').textContent = pose.sinHorizon.toExponential(2);
  $('m-elev').textContent = `${(Math.asin(Math.max(-1, Math.min(1, dot(pose.forward, pose.gravityUp)))) * RAD2DEG).toFixed(2)}°`;

  metricHistory.roll.push(pose.rollDeg ?? 0);
  metricHistory.step.push(pose.stepDeg ?? 0);
  metricHistory.overlap.push(pose.fovOverlap == null ? 1 : pose.fovOverlap);
  for (const k of Object.keys(metricHistory)) {
    if (metricHistory[k].length > 220) metricHistory[k].shift();
  }
}

function drawChart() {
  const w = chartCanvas.width;
  const h = chartCanvas.height;
  chartCtx.clearRect(0, 0, w, h);
  chartCtx.fillStyle = 'rgba(6,10,20,0.6)';
  chartCtx.fillRect(0, 0, w, h);

  chartCtx.strokeStyle = 'rgba(94,132,190,0.16)';
  chartCtx.lineWidth = 1;
  for (let i = 1; i < 4; i += 1) {
    chartCtx.beginPath();
    chartCtx.moveTo(0, (h * i) / 4);
    chartCtx.lineTo(w, (h * i) / 4);
    chartCtx.stroke();
  }

  const n = metricHistory.roll.length;
  if (n > 1) {
    // Roll: ±5° full scale.
    const rollMax = 5;
    const drawSeries = (arr, color, mapFn) => {
      chartCtx.strokeStyle = color;
      chartCtx.lineWidth = 1.5;
      chartCtx.beginPath();
      for (let i = 0; i < n; i += 1) {
        const x = (i / (n - 1)) * w;
        const y = h - mapFn(arr[i]) * h;
        if (i === 0) chartCtx.moveTo(x, y); else chartCtx.lineTo(x, y);
      }
      chartCtx.stroke();
    };
    drawSeries(metricHistory.roll, '#ffd479', (v) => Math.max(0, Math.min(1, (v / rollMax + 1) / 2)));
    drawSeries(metricHistory.step, '#ff8a5c', (v) => Math.max(0, Math.min(1, v / 12)));
    drawSeries(metricHistory.overlap, '#5fe0a4', (v) => 1 - Math.max(0, Math.min(1, v)));
  }

  chartCtx.fillStyle = 'rgba(143,160,189,0.85)';
  chartCtx.font = '9px monospace';
  chartCtx.fillText('±5°', 4, 11);
  chartCtx.fillText('重叠 0', 4, h - 4);
}

/**
 * "Gravity lens" projection: where does the active local up appear in the
 * current camera view, and where is the local horizon? This is computed from
 * the real pose basis, not from a skybox rotation.
 */
function drawLens(pose) {
  const s = lensCanvas.width;
  const ctx = lensCtx;
  ctx.clearRect(0, 0, s, s);
  ctx.fillStyle = 'rgba(6,10,20,0.65)';
  ctx.fillRect(0, 0, s, s);

  const cx = s / 2;
  const cy = s / 2;
  const r = s * 0.4;
  ctx.strokeStyle = 'rgba(94,132,190,0.45)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
  ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
  ctx.stroke();

  const dirToScreen = (worldDir) => {
    const z = dot(worldDir, pose.forward);
    if (z <= 0.05) return null;
    const x = dot(worldDir, pose.right) / z;
    const y = dot(worldDir, pose.up) / z;
    return { x: cx + (x / Math.tan((state.fov * DEG2RAD) / 2)) * (s / 2) * 0.62, y: cy - (y / Math.tan((state.fov * DEG2RAD) / 2)) * (s / 2) * 0.62, z };
  };

  // Local horizon circle: all directions perpendicular to the gravity up.
  const U = pose.gravityUp;
  const e1 = normalize(cross(U, Math.abs(U.y) < 0.9 ? v3(0, 1, 0) : v3(1, 0, 0)));
  const e2 = cross(U, e1);
  ctx.strokeStyle = 'rgba(255,212,121,0.75)';
  ctx.lineWidth = 1.4;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  let started = false;
  for (let i = 0; i <= 180; i += 1) {
    const a = (i / 180) * Math.PI * 2;
    const dir = add(scale(e1, Math.cos(a)), scale(e2, Math.sin(a)));
    const p = dirToScreen(dir);
    if (!p) { started = false; continue; }
    if (!started) { ctx.moveTo(p.x, p.y); started = true; } else ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  ctx.setLineDash([]);

  // Local up marker.
  const upPt = dirToScreen(U);
  if (upPt) {
    ctx.fillStyle = '#ffd479';
    ctx.beginPath();
    ctx.arc(upPt.x, upPt.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,212,121,0.4)';
    ctx.beginPath();
    ctx.arc(upPt.x, upPt.y, 10, 0, Math.PI * 2);
    ctx.stroke();
  } else {
    ctx.fillStyle = '#ff8a5c';
    ctx.font = '10px monospace';
    ctx.fillText('局部上方向在视野后方', 10, s - 12);
  }

  // View centre.
  ctx.fillStyle = 'rgba(92,184,255,0.9)';
  ctx.beginPath();
  ctx.arc(cx, cy, 2, 0, Math.PI * 2);
  ctx.fill();

  $('lens-hint').textContent = pose.rollDeg == null
    ? '当前视线与局部上方向重合：地平线退化为点，采用上一帧切向连续回退。'
    : `局部上方向在屏幕方位 ${(Math.atan2(upPt ? upPt.y - cy : 0, upPt ? upPt.x - cx : 0) * RAD2DEG).toFixed(0)}°，地平线滚转 ${pose.rollDeg.toFixed(2)}°。`;
}

// ---------------------------------------------------------------------------
// Anomaly toasts
// ---------------------------------------------------------------------------
const stack = $('anomaly-stack');
function toastAnomaly(a) {
  const el = document.createElement('div');
  el.className = `anomaly ${a.level}`;
  el.innerHTML = `<span class="code">${a.code}</span><span>${a.message}</span>`;
  stack.appendChild(el);
  while (stack.children.length > 4) stack.removeChild(stack.firstChild);
  setTimeout(() => {
    el.style.transition = 'opacity 0.4s, transform 0.4s';
    el.style.opacity = '0';
    el.style.transform = 'translateY(-6px)';
    setTimeout(() => el.remove(), 420);
  }, 3200);
}

function publishAnomalies(pose) {
  const keys = new Set();
  for (const a of pose.anomalies) {
    const key = `${a.code}`;
    keys.add(key);
    if (!state.lastAnomalyKeys.has(key)) toastAnomaly(a);
  }
  state.lastAnomalyKeys = keys;
}

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------
function exportSamples() {
  const payload = {
    generatedAt: new Date().toISOString(),
    mode: state.mode,
    fovDeg: state.fov,
    allowPoleCrossing: state.allowPoleCrossing,
    samples: state.samples,
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `gravity-lens-${state.mode}-${Date.now()}.json`;
  a.click();
  URL.revokeObjectURL(a.href);
}

// ---------------------------------------------------------------------------
// Resize
// ---------------------------------------------------------------------------
function resize() {
  const w = window.innerWidth;
  const h = window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  rig.setFov(state.fov, camera.aspect);
}
window.addEventListener('resize', resize);
resize();

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
let lastTime = performance.now();
let fpsAccum = 0;
let fpsFrames = 0;

function updateScenario(dt) {
  const run = state.scenarioRun;
  if (!run) return null;
  if (!state.playing) return run.records[Math.min(run.records.length - 1, Math.floor(state.scenarioProgress * (run.records.length - 1)))];
  state.scenarioClock += dt;
  const fps = 60;
  const idx = Math.min(run.records.length - 1, Math.floor(state.scenarioClock * fps));
  state.scenarioProgress = idx / (run.records.length - 1);
  $('scenario-progress').value = String(state.scenarioProgress);
  $('scenario-state').textContent = `帧 ${idx + 1}/${run.records.length} · t=${(state.scenarioProgress * 100).toFixed(0)}%`;
  if (idx >= run.records.length - 1) {
    state.playing = false;
    $('btn-play').textContent = '播放';
    refreshSummary();
  }
  return run.records[idx];
}

const CHAMBER_FACES = [
  { up: v3(0, 1, 0), point: v3(0, 0, 0) },
  { up: v3(0, -1, 0), point: v3(0, CHAMBER_HEIGHT, 0) },
  { up: v3(1, 0, 0), point: v3(-CHAMBER_HALF, CHAMBER_HEIGHT / 2, 0) },
  { up: v3(-1, 0, 0), point: v3(CHAMBER_HALF, CHAMBER_HEIGHT / 2, 0) },
  { up: v3(0, 0, 1), point: v3(0, CHAMBER_HEIGHT / 2, -CHAMBER_HALF) },
  { up: v3(0, 0, -1), point: v3(0, CHAMBER_HEIGHT / 2, CHAMBER_HALF) },
];

/** Point on the nearest real chamber face whose inward normal matches `up`. */
function surfaceAnchorFor(up) {
  let best = CHAMBER_FACES[0];
  let bestDot = -Infinity;
  for (const f of CHAMBER_FACES) {
    const d = dot(f.up, up);
    if (d > bestDot) { bestDot = d; best = f; }
  }
  return best.point;
}

function applyScenarioRecord(rec, run, viewPos) {
  const scenario = scenarios.find((x) => x.id === run.scenario);
  state.gravityUp = rec.gravityUp;
  const pose = {
    ...rig.lastPose,
    forward: rec.forward,
    up: rec.up,
    right: rec.right,
    quaternion: rec.quat,
    gravityUp: rec.gravityUp,
    yawDeg: rec.yawDeg,
    pitchDeg: rec.pitchDeg,
    sinHorizon: rec.sinHorizon,
    solveMode: rec.solveMode,
    rollDeg: rec.rollDeg,
    horizonInverted: rec.horizonInverted ?? false,
    orthoErrorDeg: rec.orthoErrorDeg,
    stepDeg: rec.stepDeg,
    fovOverlap: rec.fovOverlap,
    anomalies: rec.anomalies,
    referenceForward: rig.refForward,
  };
  const target = rec.playerPos
    || add(surfaceAnchorFor(rec.gravityUp), scale(rec.gravityUp, PLAYER_HEIGHT));
  // Smoothly follow the simulated character so the playback camera and the
  // character model stay together without teleporting per frame.
  if (!state.scenarioViewPos) state.scenarioViewPos = target;
  const k = 0.18;
  state.scenarioViewPos = add(
    state.scenarioViewPos,
    scale(sub(target, state.scenarioViewPos), k),
  );
  state.playerPos = state.scenarioViewPos;
  const h = normalize(sub(rec.forward, scale(rec.gravityUp, dot(rec.forward, rec.gravityUp))));
  const walkDir = rec.charFacing ? normalize(rec.charFacing) : h;
  state.characterYawVec = len(walkDir) > 1e-5
    ? walkDir
    : normalize(sub(rec.right, scale(rec.gravityUp, dot(rec.right, rec.gravityUp))));
  return pose;
}

function tick() {
  const now = performance.now();
  const dt = Math.min(0.05, (now - lastTime) / 1000);
  lastTime = now;
  fpsAccum += dt;
  fpsFrames += 1;

  const f = (keys.has('KeyW') || keys.has('ArrowUp') ? 1 : 0) - (keys.has('KeyS') || keys.has('ArrowDown') ? 1 : 0);
  const r = (keys.has('KeyD') || keys.has('ArrowRight') ? 1 : 0) - (keys.has('KeyA') || keys.has('ArrowLeft') ? 1 : 0);
  state.input.f = f;
  state.input.r = r;

  let pose;
  if (state.mode === 'scenario' && state.scenarioRun) {
    const rec = updateScenario(dt);
    pose = rec ? applyScenarioRecord(rec, state.scenarioRun) : rig.lastPose;
  } else {
    if (state.mode === 'planet') {
      setGravityUp(normalize(sub(state.playerPos, v3(0, 0, 0))));
    }
    let heading = normSafe(sub(rig.lastPose.forward, scale(state.gravityUp, dot(rig.lastPose.forward, state.gravityUp))));
    if (len(heading) < 1e-5) heading = rig.refForward;
    if (f !== 0 || r !== 0) {
      const right = normalize(cross(heading, state.gravityUp));
      moveOnSurface(heading, right, dt);
    } else {
      state.moving = false;
    }
    pose = rig.step({ gravityUp: state.gravityUp });
  }

  const poseForView = pose;
  const camPos = cameraPositionFromPivot(poseForView, state.playerPos, {
    boom: state.boom,
    height: 1.35,
    surfaceUp: state.gravityUp,
    minClearance: 0.6,
  });
  camera.position.set(camPos.x, camPos.y, camPos.z);
  const m = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(poseForView.right.x, poseForView.right.y, poseForView.right.z),
    new THREE.Vector3(poseForView.up.x, poseForView.up.y, poseForView.up.z),
    new THREE.Vector3(poseForView.back.x, poseForView.back.y, poseForView.back.z),
  );
  camera.quaternion.setFromRotationMatrix(m);
  camera.fov = state.fov;
  camera.updateProjectionMatrix();

  updateCharacter(dt);

  updateGhosts(camPos, poseForView);
  updateReadouts(poseForView);
  publishAnomalies(poseForView);
  drawChart();
  drawLens(poseForView);

  state.samples.push({
    t: performance.now(),
    mode: state.mode,
    gravityUp: poseForView.gravityUp,
    forward: poseForView.forward,
    up: poseForView.up,
    right: poseForView.right,
    quaternion: poseForView.quaternion,
    yawDeg: poseForView.yawDeg,
    pitchDeg: poseForView.pitchDeg,
    solveMode: poseForView.solveMode,
    sinHorizon: poseForView.sinHorizon,
    orthoErrorDeg: poseForView.orthoErrorDeg,
    rollDeg: poseForView.rollDeg,
    stepDeg: poseForView.stepDeg,
    fovOverlap: poseForView.fovOverlap,
    anomalies: poseForView.anomalies,
  });
  if (state.samples.length > 1200) state.samples.shift();
  $('sample-count').textContent = String(state.samples.length);

  // Keep the sun anchored to the player so the lit hemisphere is the one we
  // are standing on; in planet mode this is what makes the surface readable.
  // Keep the sun anchored to the player so the lit hemisphere is the one we
  // are standing on; only re-anchor when the player or gravity actually moves,
  // to avoid re-rendering the shadow map every frame.
  const anchor = state.sunAnchor;
  const moved = !anchor
    || Math.hypot(
      state.playerPos.x - anchor.pos.x,
      state.playerPos.y - anchor.pos.y,
      state.playerPos.z - anchor.pos.z,
    ) > 0.6
    || dot(anchor.up, state.gravityUp) < 0.9999;
  if (moved) {
    const sunOffset = scale(state.gravityUp, 60);
    const sunSide = scale(normalize(cross(state.gravityUp, v3(0.4, 0, 0.9))), 42);
    world.sun.position.set(
      state.playerPos.x + sunOffset.x + sunSide.x,
      state.playerPos.y + sunOffset.y + Math.abs(sunSide.y) + 18,
      state.playerPos.z + sunOffset.z + sunSide.z,
    );
    world.sun.target.position.set(state.playerPos.x, state.playerPos.y, state.playerPos.z);
    world.sun.target.updateMatrixWorld();
    state.sunAnchor = { pos: state.playerPos, up: state.gravityUp };
  }
  // Light follows the view so every gravity orientation stays legible.
  world.headlight.position.set(camPos.x, camPos.y, camPos.z);
  world.headlight.target.position.set(
    camPos.x + poseForView.forward.x * 10,
    camPos.y + poseForView.forward.y * 10,
    camPos.z + poseForView.forward.z * 10,
  );
  world.headlight.target.updateMatrixWorld();
  world.stars.position.set(camPos.x * 0.92, camPos.y * 0.92, camPos.z * 0.92);
  if (state.mode === 'planet') {
    scene.fog.density = 0.0022;
  } else {
    scene.fog.density = 0.0065;
  }

  renderer.render(scene, camera);

  if (fpsAccum > 0.5) {
    $('status-perf').textContent = `${(fpsFrames / fpsAccum).toFixed(0)} fps · draw ${renderer.info.render.calls} · tris ${renderer.info.render.triangles}`;
    fpsAccum = 0;
    fpsFrames = 0;
  }
  requestAnimationFrame(tick);
}

function normSafe(v) {
  const l = len(v);
  return l < 1e-6 ? v3(0, 0, 0) : scale(v, 1 / l);
}

// Boot
applyMode('planet');
setCharacterYaw(0);
rig.setFov(state.fov, camera.aspect);
requestAnimationFrame(tick);

// Expose a small test hook for headless continuity checks.
window.__gravityLens = {
  state,
  rig,
  getPose: () => rig.lastPose,
  setGravity: (up) => setGravityUp(up, { reseat: true }),
  setGravityId: (id) => {
    const s = chamberSurfaces().find((x) => x.id === id);
    if (s) {
      state.gravityId = s.id;
      state.playerPos = add(s.stand, scale(s.up, PLAYER_HEIGHT));
      setGravityUp(s.up, { reseat: true });
      rig.setYawPitch(0, 8 * DEG2RAD);
      rig.resetContinuity();
      syncGravityButtons();
    }
  },
  setLook: (yawDeg, pitchDeg) => rig.setYawPitch(yawDeg * DEG2RAD, pitchDeg * DEG2RAD),
  setCharacterYawDeg: (deg) => setCharacterYaw(deg * DEG2RAD),
  camera,
  renderer,
  runNamedScenario: (id) => {
    const s = scenarios.find((x) => x.id === id);
    const run = runScenario(s, { fovDeg: state.fov, aspect: camera.aspect });
    return { stats: run.stats, evaluation: evaluate(run.stats) };
  },
  latestSample: () => state.samples[state.samples.length - 1],
  surfaces: chamberSurfaces,
  THREE,
};
