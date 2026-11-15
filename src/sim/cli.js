#!/usr/bin/env node
/**
 * Auxiliary numeric CLI for the Gravity Lens demo.
 *
 * The interactive demo (npm run dev -> http://localhost:5173) is the primary
 * entry point. This CLI runs the exact same solver and scenario definitions
 * headlessly and writes numeric records for inspection or regression checks.
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import { resolve } from 'node:path';
import { allScenarios, runScenario, evaluate } from './scenarios.js';

function parseArgs(argv) {
  const opts = { ids: [], out: null, full: false, json: false, help: false };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--all') opts.ids = allScenarios().map((s) => s.id);
    else if (a === '--out') { opts.out = argv[i + 1]; i += 1; } else if (a === '--full') opts.full = true;
    else if (a === '--json') opts.json = true;
    else if (a === '--help' || a === '-h') opts.help = true;
    else if (a.startsWith('--out=')) opts.out = a.slice(6);
    else if (!a.startsWith('-')) opts.ids.push(a);
  }
  return opts;
}

function fmt(x, d = 3) {
  return Number.isFinite(x) ? x.toFixed(d) : String(x);
}

function vec(v) {
  return `(${fmt(v.x, 2)}, ${fmt(v.y, 2)}, ${fmt(v.z, 2)})`;
}

function printReport(run) {
  const ev = evaluate(run.stats);
  const lines = [];
  lines.push(`\n=== ${run.scenario} — ${run.label} ${ev.pass ? '[PASS]' : '[FAIL]'} ===`);
  lines.push(`frames=${run.stats.frames}  modes: cross=${run.stats.modeCounts.cross} blend=${run.stats.modeCounts.blend} transport=${run.stats.modeCounts.transport}`);
  lines.push(`  max ortho error   ${run.stats.maxOrthoErrorDeg.toExponential(2)}°`);
  lines.push(`  max |roll|        ${fmt(run.stats.maxRollDeg, 4)}°  (defined frames ${run.stats.rollSamples}, inverted-view frames ${run.stats.invertedFrames})`);
  lines.push(`  max frame step    ${fmt(run.stats.maxStepDeg)}°`);
  lines.push(`  min frustum ovlp  ${fmt(run.stats.minFovOverlap * 100, 1)}%`);
  lines.push(`  max right turn    ${fmt(run.stats.maxRightTurnWhileForwardStillDeg, 2)}° (forward still)`);
  lines.push(`  gravity switches  ${run.stats.teleports}`);
  for (const c of ev.checks) lines.push(`  ${c.ok ? 'PASS' : 'FAIL'}  ${c.name}: ${c.detail}`);
  if (run.stats.anomalies.length) {
    lines.push(`  anomalies (${run.stats.anomalies.length}):`);
    for (const a of run.stats.anomalies.slice(0, 8)) {
      lines.push(`    [${a.level}] ${a.code} @frame ${a.i}: ${a.message}`);
    }
  }
  lines.push('  sample frames (full numeric records):');
  const R = run.allRecords || run.records;
  const picks = [0, Math.floor(R.length * 0.25), Math.floor(R.length * 0.5), Math.floor(R.length * 0.75), R.length - 1];
  for (const idx of picks) {
    const r = R[idx];
    lines.push(`    #${r.i} up=${vec(r.up)} fwd=${vec(r.forward)} gUp=${vec(r.gravityUp)} yaw=${fmt(r.yawDeg, 1)}° pitch=${fmt(r.pitchDeg, 1)}° mode=${r.solveMode} Δ=${fmt(r.stepDeg, 3)}° ov=${fmt((r.fovOverlap ?? 1) * 100, 1)}%`);
  }
  return lines.join('\n');
}

function main() {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    console.log(`Gravity Lens numeric CLI (auxiliary; the graphical demo is the primary entry)

Usage:
  node src/sim/cli.js --all                 run every scenario and print a report
  node src/sim/cli.js sphere-walk wall-turn run the named scenarios
  node src/sim/cli.js --all --out artifacts write one JSON per scenario plus index.json
  node src/sim/cli.js --all --json          print machine-readable JSON only
  node src/sim/cli.js --all --full          include every frame in the JSON

Scenario ids: ${allScenarios().map((s) => s.id).join(', ')}`);
    return;
  }
  const scenarios = allScenarios().filter((s) => opts.ids.length === 0 || opts.ids.includes(s.id));
  if (!scenarios.length) {
    console.error(`No matching scenario. Known: ${allScenarios().map((s) => s.id).join(', ')}`);
    process.exitCode = 2;
    return;
  }

  const runs = scenarios.map((s) => {
    const run = runScenario(s);
    const stride = Math.max(1, Math.floor(run.records.length / 60));
    return {
      scenario: run.scenario,
      label: run.label,
      stats: run.stats,
      evaluation: evaluate(run.stats),
      records: opts.full ? run.records : run.records.filter((_, i) => i % stride === 0),
      allRecords: run.records,
    };
  });

  if (opts.json) {
    console.log(JSON.stringify({
      generatedAt: new Date().toISOString(),
      runs: runs.map(({ allRecords, ...rest }) => ({ ...rest, records: rest.records })),
    }, null, 2));
  } else {
    for (const r of runs) console.log(printReport(r));
    const allPass = runs.every((r) => r.evaluation.pass);
    console.log(`\n${allPass ? 'ALL SCENARIOS PASS' : 'SOME SCENARIOS FAILED'} (${runs.length} run)`);
  }

  if (opts.out) {
    const base = resolve(opts.out);
    mkdirSync(base, { recursive: true });
    const index = {
      generatedAt: new Date().toISOString(),
      note: 'Numerical records from the same solver used by the Gravity Lens graphical demo.',
      runs: [],
    };
    for (const r of runs) {
      const fullRun = runScenario(allScenarios().find((s) => s.id === r.scenario));
      const file = `${r.scenario}.json`;
      writeFileSync(resolve(base, file), `${JSON.stringify({
        scenario: r.scenario,
        label: r.label,
        stats: fullRun.stats,
        evaluation: evaluate(fullRun.stats),
        records: fullRun.records,
      }, null, 2)}\n`);
      index.runs.push({
        scenario: r.scenario,
        label: r.label,
        file,
        frames: fullRun.records.length,
        pass: r.evaluation.pass,
        maxOrthoErrorDeg: fullRun.stats.maxOrthoErrorDeg,
        maxRollDeg: fullRun.stats.maxRollDeg,
        maxStepDeg: fullRun.stats.maxStepDeg,
        minFovOverlap: fullRun.stats.minFovOverlap,
        modeCounts: fullRun.stats.modeCounts,
      });
    }
    writeFileSync(resolve(base, 'index.json'), `${JSON.stringify(index, null, 2)}\n`);
    console.log(`\nWrote ${runs.length} scenario files + index.json to ${base}`);
  } else if (!opts.json) {
    console.log('\nTip: add --out artifacts to write full per-frame JSON records.');
  }
}

main();
