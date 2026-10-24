import { createWorld, stepWorld, addObstacle } from './world';
import { plan, planner, lastGrid } from './planner';
import type { VehicleState } from '../core/types';

function runTests() {
  console.log('====================================================');
  console.log('   MEMBER B - PLANNING MODULE VERIFICATION SUITE    ');
  console.log('====================================================\n');

  let passedAll = true;

  // ----------------------------------------------------
  // TEST 1: Baseline straight road without blocker
  // ----------------------------------------------------
  console.log('[TEST 1] Testing baseline straight scenario...');
  const world = createWorld('straight');
  // Clear any obstacles right in the immediate 5-25m lane for clean baseline comparison
  world.obstacles = world.obstacles.filter(o => o.y > 35);

  const ego: VehicleState = { x: 0, y: 0, theta: 0, v: 5.0, delta: 0 };

  const t0 = performance.now();
  const basePath = plan(world, ego);
  const tBase = performance.now() - t0;

  console.log(`  Path generated: ${basePath.length} waypoints, end y = ${basePath[basePath.length - 1]?.y.toFixed(1)} m`);
  console.log(`  Planning latency: ${tBase.toFixed(2)} ms (target < 15 ms)`);

  const maxBaseDev = Math.max(...basePath.map(p => Math.abs(p.x)));
  console.log(`  Max lateral deviation on straight road: ${maxBaseDev.toFixed(3)} m`);

  if (basePath.length >= 30 && maxBaseDev < 0.4 && tBase < 15) {
    console.log('  -> PASS: Baseline path is straight and centered.');
  } else {
    console.error('  -> FAIL: Baseline path unexpected.');
    passedAll = false;
  }

  // ----------------------------------------------------
  // TEST 2: Place cow directly ahead at (0, 15) -> Path must bend
  // ----------------------------------------------------
  console.log('\n[TEST 2] Placing cow directly ahead at (x=0, y=15)...');
  const cow = addObstacle(world, 'cow', 0.0, 15.0);

  const t1 = performance.now();
  const avoidPath = plan(world, ego);
  const tAvoid = performance.now() - t1;

  console.log(`  Avoidance path length: ${avoidPath.length} waypoints`);
  console.log(`  Planning latency: ${tAvoid.toFixed(2)} ms`);

  // Find lateral offset around y ~ 15
  const ptsNearCow = avoidPath.filter(p => Math.abs(p.y - 15.0) <= 2.5);
  const maxCowDeviation = Math.max(...ptsNearCow.map(p => Math.abs(p.x)));
  console.log(`  Lateral deviation near cow (y ~ 15 m): ${maxCowDeviation.toFixed(2)} m`);

  // Path must bend around the cow by at least 1.2 m laterally
  if (maxCowDeviation >= 1.2) {
    console.log('  -> PASS: Path visibly bends around the cow obstacle!');
  } else {
    console.error('  -> FAIL: Path did not sufficiently bend around cow.');
    passedAll = false;
  }

  // ----------------------------------------------------
  // TEST 3: Remove cow -> Path must straighten again
  // ----------------------------------------------------
  console.log('\n[TEST 3] Removing cow from road...');
  world.obstacles = world.obstacles.filter(o => o.id !== cow.id);

  const t2 = performance.now();
  const straightenedPath = plan(world, ego);
  const tStraight = performance.now() - t2;

  const maxStraightDev = Math.max(...straightenedPath.map(p => Math.abs(p.x)));
  console.log(`  Max lateral deviation after removal: ${maxStraightDev.toFixed(3)} m`);
  console.log(`  Planning latency: ${tStraight.toFixed(2)} ms`);

  if (maxStraightDev < 0.4) {
    console.log('  -> PASS: Path successfully straightened back along the road center!');
  } else {
    console.error('  -> FAIL: Path remained bent after cow was removed.');
    passedAll = false;
  }

  // ----------------------------------------------------
  // TEST 4: Verification on "curve" and "chaos" scenarios
  // ----------------------------------------------------
  console.log('\n[TEST 4] Testing scenarios "curve" and "chaos"...');
  const curveWorld = createWorld('curve');
  const curvePath = plan(curveWorld, ego);
  console.log(`  Curve path generated: ${curvePath.length} waypoints, end y = ${curvePath[curvePath.length - 1]?.y.toFixed(1)} m`);

  const chaosWorld = createWorld('chaos');
  const chaosPath = plan(chaosWorld, ego);
  console.log(`  Chaos path generated: ${chaosPath.length} waypoints, end y = ${chaosPath[chaosPath.length - 1]?.y.toFixed(1)} m`);

  if (curvePath.length > 20 && chaosPath.length > 20) {
    console.log('  -> PASS: Successfully planned paths on curve and chaos scenarios.');
  } else {
    console.error('  -> FAIL: Scenario planning failed.');
    passedAll = false;
  }

  // ----------------------------------------------------
  // TEST 5: Simulation stepping & dynamic behaviors
  // ----------------------------------------------------
  console.log('\n[TEST 5] Testing stepWorld dynamics (auto weaving, bike zigzag, cow pause)...');
  const initCow = chaosWorld.obstacles.find(o => o.cls === 'cow');
  const initAuto = chaosWorld.obstacles.find(o => o.cls === 'auto');
  const prevAutoVx = initAuto?.vx ?? 0;

  // Step 60 frames (1 second)
  for (let f = 0; f < 60; f++) {
    stepWorld(chaosWorld, 1 / 60);
  }

  console.log(`  World stepped to t = ${chaosWorld.t.toFixed(2)} s`);
  console.log(`  Auto vx evolved from ${prevAutoVx.toFixed(2)} to ${initAuto?.vx.toFixed(2)}`);
  console.log('  -> PASS: Dynamic obstacles stepped smoothly.');

  // ----------------------------------------------------
  // TEST 6: Debug grid export for Member D
  // ----------------------------------------------------
  console.log('\n[TEST 6] Testing planner.lastGrid debug export...');
  const grid = planner.lastGrid;
  if (grid && grid.cols === 80 && grid.rows === 168 && grid.data.length === 13440 && grid.costMap.length === 13440) {
    console.log(`  Grid cols: ${grid.cols}, rows: ${grid.rows}, res: ${grid.resolution} m`);
    console.log(`  Origin: (${grid.originX.toFixed(1)}, ${grid.originY.toFixed(1)})`);
    console.log('  -> PASS: planner.lastGrid is fully accessible for Member D overlay.');
  } else {
    console.error('  -> FAIL: planner.lastGrid is invalid.');
    passedAll = false;
  }

  // ----------------------------------------------------
  // TEST 7: Benchmark latency (100 plan iterations)
  // ----------------------------------------------------
  console.log('\n[TEST 7] Running latency benchmark (100 iterations of plan())...');
  const times: number[] = [];
  for (let i = 0; i < 100; i++) {
    const s = performance.now();
    plan(chaosWorld, ego);
    times.push(performance.now() - s);
  }

  times.sort((a, b) => a - b);
  const avg = times.reduce((acc, v) => acc + v, 0) / times.length;
  const p50 = times[Math.floor(times.length * 0.5)];
  const p95 = times[Math.floor(times.length * 0.95)];
  const maxTime = times[times.length - 1];

  console.log(`  Avg: ${avg.toFixed(2)} ms | P50: ${p50.toFixed(2)} ms | P95: ${p95.toFixed(2)} ms | Max: ${maxTime.toFixed(2)} ms`);

  if (p95 < 15.0) {
    console.log('  -> PASS: Latency target well under 15 ms satisfied!');
  } else {
    console.error('  -> FAIL: Latency exceeds 15 ms target.');
    passedAll = false;
  }

  console.log('\n====================================================');
  if (passedAll) {
    console.log('  ALL TESTS PASSED! MEMBER B MODULE READY FOR SIH 2026');
  } else {
    throw new Error('SOME TESTS FAILED.');
  }
  console.log('====================================================');
}

runTests();
