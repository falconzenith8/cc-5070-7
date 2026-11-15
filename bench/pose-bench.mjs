#!/usr/bin/env node
/**
 * Pose-update benchmark: frozen gravity-pose baseline vs optimised solver.
 *
 * Both rigs execute for real in this process. For each load we warm both up,
 * then time them in alternating turns and report the median so a single GC
 * pause cannot flatter either side. Angle error and update cost are reported
 * together: a speedup only counts if the pose metrics match the baseline.
 *
 * Usage:
 *   node bench/pose-bench.mjs            # human-readable table
 *   node bench/pose-bench.mjs --json     # machine-readable, for artifacts
 *   node bench/pose-bench.mjs --save     # also write artifacts/pose-bench.json
 */
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { loadavg, cpus } from 'node:os';
import { GravityCameraRig as BaselineRig } from '../src/core/rigBaseline.js';
import { GravityCameraRig as OptimizedRig, frustumOverlap } from '../src/core/rig.js';
import { quatAngle } from '../src/core/quat.js';
import { LOADS, runClip, median } from './loads.js';

const REPS = 5;
const ROUNDS = 5;
const WARMUP = 2;

/**
 * Cost breakdown of one full optimised step, measured by timing the real
 * components in isolation (not estimated from source size):
 *   - solveGeometry : the pure pose solve (heading/forward/horizon/quat/metrics)
 *   - frustumOverlap: the metric that compares consecutive frusta
 *   - detectAnomalies: the anomaly scan over the solved pose
 * `fullStep` is a real `rig.step()` on a moving-gravity clip, which forces the
 * full path (no cache hit). Per-component times do not sum exactly to the step
 * because the harness adds a little plumbing around each call; they are
 * reported as measured per-call cost, and the share is that cost over fullStep.
 */
function breakdown() {
  const load = LOADS.find((l) => l.id === 'slow-gravity');
  const clip = load.clip(load.frames);
  const rig = new OptimizedRig({ seedUp: load.seed, allowPoleCrossing: true, fps: 60 });
  rig.setYawPitch(0.2, (load.pitchDeg * Math.PI) / 180);
  rig.resetContinuity();
  for (let i = 0; i < 200; i += 1) rig.step(clip[i]);
  const pose = rig.lastPose;
  const up = rig.up;

  const time = (fn, n) => {
    for (let i = 0; i < 2000; i += 1) fn();
    const t0 = process.hrtime.bigint();
    for (let i = 0; i < n; i += 1) fn();
    return Number(process.hrtime.bigint() - t0) / 1000 / n;
  };

  // Sample every component once per round and take the median, so a single
  // scheduled-out round cannot set the reported share.
  const samples = { fullStepUs: [], solveGeometryUs: [], frustumOverlapUs: [], detectAnomaliesUs: [] };
  for (let r = 0; r < 5; r += 1) {
    samples.solveGeometryUs.push(time(() => rig._solveGeometry(up), 100000));
    samples.frustumOverlapUs.push(time(() => frustumOverlap(pose, pose, rig.fovDeg, rig.aspect), 100000));
    samples.detectAnomaliesUs.push(time(() => rig.detectAnomalies(pose), 100000));
    samples.fullStepUs.push(time(() => rig.step(clip[200]), 5000));
  }
  const solveGeometryUs = median(samples.solveGeometryUs);
  const frustumOverlapUs = median(samples.frustumOverlapUs);
  const detectAnomaliesUs = median(samples.detectAnomaliesUs);
  const fullStepUs = median(samples.fullStepUs);
  const remainderUs = fullStepUs - solveGeometryUs - frustumOverlapUs - detectAnomaliesUs;
  return {
    fullStepUs,
    fullStepRangeUs: { min: Math.min(...samples.fullStepUs), max: Math.max(...samples.fullStepUs) },
    remainderUs,
    components: {
      solveGeometryUs,
      frustumOverlapUs,
      detectAnomaliesUs,
    },
    shareOfFullStep: {
      solveGeometry: solveGeometryUs / fullStepUs,
      frustumOverlap: frustumOverlapUs / fullStepUs,
      detectAnomalies: detectAnomaliesUs / fullStepUs,
      remainder: remainderUs / fullStepUs,
    },
  };
}

function compare(load) {
  const clip = load.clip(load.frames);
  for (let i = 0; i < WARMUP; i += 1) {
    runClip(BaselineRig, load, clip);
    runClip(OptimizedRig, load, clip);
  }
  const base = [];
  const opt = [];
  const roundSpeeds = [];
  let baseLast = null;
  let optLast = null;
  for (let r = 0; r < ROUNDS; r += 1) {
    // Each round is timed as a block; a whole-round spike (GC, scheduler) then
    // shows up in the spread instead of silently rewriting one round's median.
    const roundBase = [];
    const roundOpt = [];
    for (let i = 0; i < REPS; i += 1) {
      const b = runClip(BaselineRig, load, clip);
      const o = runClip(OptimizedRig, load, clip);
      base.push(b.usPerFrame);
      opt.push(o.usPerFrame);
      roundBase.push(b.usPerFrame);
      roundOpt.push(o.usPerFrame);
      baseLast = b;
      optLast = o;
    }
    roundSpeeds.push(median(roundBase) / median(roundOpt));
  }
  const baseUs = median(base);
  const optUs = median(opt);
  const roundMedians = roundSpeeds.sort((a, b) => a - b);
  return {
    id: load.id,
    frames: clip.length,
    baselineUsPerFrame: baseUs,
    optimizedUsPerFrame: optUs,
    // Median of per-round speedup ratios, so the headline sits inside the range.
    speedup: median(roundMedians),
    // Round medians are independent estimates of the same ratio; their range is
    // the honest noise band on this machine.
    speedupRounds: roundMedians.map((v) => Number(v.toFixed(2))),
    speedupRange: {
      min: Number(roundMedians[0].toFixed(2)),
      max: Number(roundMedians[roundMedians.length - 1].toFixed(2)),
    },
    angleError: {
      maxStepDegBaseline: baseLast.maxStepDeg,
      maxStepDegOptimized: optLast.maxStepDeg,
      maxOrthoErrorDegBaseline: baseLast.maxOrthoErrorDeg,
      maxOrthoErrorDegOptimized: optLast.maxOrthoErrorDeg,
      maxRollDegBaseline: baseLast.maxRollDeg,
      maxRollDegOptimized: optLast.maxRollDeg,
    },
    solveModes: {
      transportBaseline: baseLast.transport,
      transportOptimized: optLast.transport,
      blendBaseline: baseLast.blend,
      blendOptimized: optLast.blend,
    },
    // Whole-pose angular difference between the two runs at the final frame.
    finalPoseDeltaDeg: quatAngle(baseLast.finalPose.quaternion, optLast.finalPose.quaternion) * (180 / Math.PI),
  };
}

function main() {
  const args = process.argv.slice(2);
  const results = LOADS.map(compare);
  const cost = breakdown();
  const env = {
    note: 'Synthetic micro-benchmark on a shared machine; round ranges show the noise band, not a guarantee.',
    loadavgAtEnd: Math.round(loadavg()[0] * 100) / 100,
    cpuCount: cpus().length,
  };
  if (args.includes('--json')) {
    const payload = {
      generatedAt: new Date().toISOString(),
      rigs: { baseline: 'src/core/rigBaseline.js', optimized: 'src/core/rig.js' },
      env,
      results,
      costBreakdown: cost,
    };
    console.log(JSON.stringify(payload, null, 2));
  } else {
    console.log('\nPose-update benchmark — baseline vs optimised (same inputs, same process)\n');
    console.log('load                     baseline   optimised   speedup   round range   maxStep Δ      ortho Δ     final pose Δ');
    for (const r of results) {
      const stepDelta = Math.abs(r.angleError.maxStepDegBaseline - r.angleError.maxStepDegOptimized);
      const orthoDelta = Math.abs(r.angleError.maxOrthoErrorDegBaseline - r.angleError.maxOrthoErrorDegOptimized);
      console.log(
        `${r.id.padEnd(24)} ${r.baselineUsPerFrame.toFixed(2).padStart(7)}us  ${r.optimizedUsPerFrame.toFixed(2).padStart(7)}us  ${r.speedup.toFixed(2).padStart(6)}x  `
        + `${`${r.speedupRange.min.toFixed(2)}-${r.speedupRange.max.toFixed(2)}x`.padStart(11)}  `
        + `${stepDelta.toExponential(1).padStart(11)}°  ${orthoDelta.toExponential(1).padStart(10)}°  ${r.finalPoseDeltaDeg.toExponential(1).padStart(12)}°`,
      );
    }
    console.log('\nAngle error columns are |optimised - baseline| for the same run; 0 means the poses match.');
    console.log('Round range is the min..max of per-round median speedups, i.e. this machine\'s noise band.');
    console.log(`Machine at end: loadavg ${env.loadavgAtEnd} on ${env.cpuCount} cpus.`);

    console.log('\nOptimised step cost breakdown (measured, moving-gravity full path):');
    console.log(`  full step            ${cost.fullStepUs.toFixed(2)} us  (round min..max ${cost.fullStepRangeUs.min.toFixed(2)}..${cost.fullStepRangeUs.max.toFixed(2)} us)`);
    console.log(`  solve geometry       ${cost.components.solveGeometryUs.toFixed(2)} us  (${(cost.shareOfFullStep.solveGeometry * 100).toFixed(0)}% of step)`);
    console.log(`  frustum overlap      ${cost.components.frustumOverlapUs.toFixed(2)} us  (${(cost.shareOfFullStep.frustumOverlap * 100).toFixed(0)}% of step)`);
    console.log(`  anomaly scan         ${cost.components.detectAnomaliesUs.toFixed(2)} us  (${(cost.shareOfFullStep.detectAnomalies * 100).toFixed(0)}% of step)`);
    console.log(`  record + history     ${cost.remainderUs.toFixed(2)} us  (${(cost.shareOfFullStep.remainder * 100).toFixed(0)}% of step, = full step minus the three above)`);
  }
  if (args.includes('--save')) {
    const file = resolve('artifacts/pose-bench.json');
    const payload = {
      generatedAt: new Date().toISOString(),
      note: 'Both rigs executed for real; timings are medians of alternating runs. Components timed in isolation per round, then medianed.',
      env,
      results,
      costBreakdown: cost,
    };
    writeFileSync(file, `${JSON.stringify(payload, null, 2)}\n`);
    console.log(`\nWrote ${file}`);
  }
}

main();
