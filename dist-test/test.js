// src/sim/vehicle.ts
var WHEELBASE = 2.7;
var STEER_LIMIT = 0.6;
var STEER_RATE_LIMIT = 0.8;
var ACCEL_MIN = -6;
var ACCEL_MAX = 3;
function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}
function normalizeAngle(angle) {
  let a = angle % (2 * Math.PI);
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
function stepVehicle(s, c, dt) {
  const targetSteer = clamp(c.steer, -STEER_LIMIT, STEER_LIMIT);
  const targetAccel = clamp(c.accel, ACCEL_MIN, ACCEL_MAX);
  const maxSteerDelta = STEER_RATE_LIMIT * dt;
  const steerDiff = targetSteer - s.delta;
  const deltaChange = clamp(steerDiff, -maxSteerDelta, maxSteerDelta);
  const nextDelta = clamp(s.delta + deltaChange, -STEER_LIMIT, STEER_LIMIT);
  const nextV = Math.max(0, s.v + targetAccel * dt);
  const omega = nextV / WHEELBASE * Math.tan(nextDelta);
  const nextTheta = normalizeAngle(s.theta + omega * dt);
  const nextX = s.x - nextV * Math.sin(nextTheta) * dt;
  const nextY = s.y + nextV * Math.cos(nextTheta) * dt;
  return {
    x: nextX,
    y: nextY,
    theta: nextTheta,
    v: nextV,
    delta: nextDelta
  };
}

// src/sim/controller.ts
var DEFAULT_CONTROLLER_CONFIG = {
  baseSpeed: 8,
  minSpeed: 1.5,
  lookaheadBase: 3,
  lookaheadGain: 0.4,
  minLookahead: 3,
  maxLookahead: 18,
  kpSpeed: 2,
  kiSpeed: 0.05,
  kdSpeed: 0.2,
  emergencyBrakeDist: 3,
  obstacleSlowdownDist: 8,
  egoLength: 4.2,
  egoWidth: 1.8
};
var speedIntegral = 0;
var lastSpeedError = 0;
function worldToVehicleLocal(wx, wy, s) {
  const dx = wx - s.x;
  const dy = wy - s.y;
  const cosT = Math.cos(s.theta);
  const sinT = Math.sin(s.theta);
  const localY = -dx * sinT + dy * cosT;
  const localX = dx * cosT + dy * sinT;
  return { localX, localY };
}
function findLookaheadPoint(s, path, lookaheadDist) {
  if (path.length === 0) {
    return { localX: 0, localY: lookaheadDist, dist: lookaheadDist };
  }
  let closestIdx = 0;
  let minCenterDist = Infinity;
  for (let i = 0; i < path.length; i++) {
    const d = Math.hypot(path[i].x - s.x, path[i].y - s.y);
    if (d < minCenterDist) {
      minCenterDist = d;
      closestIdx = i;
    }
  }
  let targetWorldX = path[closestIdx].x;
  let targetWorldY = path[closestIdx].y;
  let found = false;
  for (let i = closestIdx; i < path.length - 1; i++) {
    const p1 = path[i];
    const p2 = path[i + 1];
    const d1 = Math.hypot(p1.x - s.x, p1.y - s.y);
    const d2 = Math.hypot(p2.x - s.x, p2.y - s.y);
    if (d1 <= lookaheadDist && d2 >= lookaheadDist) {
      const denom = Math.max(1e-4, d2 - d1);
      const t = clamp((lookaheadDist - d1) / denom, 0, 1);
      targetWorldX = p1.x + t * (p2.x - p1.x);
      targetWorldY = p1.y + t * (p2.y - p1.y);
      found = true;
      break;
    }
  }
  if (!found) {
    const last = path[path.length - 1];
    targetWorldX = last.x;
    targetWorldY = last.y;
  }
  const { localX, localY } = worldToVehicleLocal(targetWorldX, targetWorldY, s);
  const actualDist = Math.hypot(localX, localY);
  return { localX, localY, dist: Math.max(0.5, actualDist) };
}
function pursuit(s, path, obstacles = [], config = DEFAULT_CONTROLLER_CONFIG) {
  const L = clamp(
    config.lookaheadBase + config.lookaheadGain * s.v,
    config.minLookahead,
    config.maxLookahead
  );
  const target = findLookaheadPoint(s, path, L);
  const actualL = Math.max(config.minLookahead, target.dist);
  const steerCmd = Math.atan2(2 * WHEELBASE * -target.localX, actualL * actualL);
  const clampedSteer = clamp(steerCmd, -0.6, 0.6);
  const pathCurvature = Math.abs(2 * -target.localX / (actualL * actualL));
  let targetSpeed = config.baseSpeed / (1 + 3.5 * pathCurvature);
  targetSpeed = Math.max(config.minSpeed, targetSpeed);
  let emergencyBrake = false;
  let minObstacleDistanceAhead = Infinity;
  const egoHalfLen = config.egoLength / 2;
  const egoHalfWidth = config.egoWidth / 2;
  for (const obs of obstacles) {
    const { localX, localY } = worldToVehicleLocal(obs.x, obs.y, s);
    const obsHalfLen = obs.l / 2;
    const obsHalfWidth = obs.w / 2;
    const corridorHalfWidth = egoHalfWidth + obsHalfWidth + 0.35;
    if (Math.abs(localX) <= corridorHalfWidth) {
      const distToFrontBumper = localY - egoHalfLen - obsHalfLen;
      if (distToFrontBumper > -0.5) {
        if (distToFrontBumper < minObstacleDistanceAhead) {
          minObstacleDistanceAhead = distToFrontBumper;
        }
        const ttc = distToFrontBumper / Math.max(0.2, s.v);
        if (distToFrontBumper <= config.emergencyBrakeDist || ttc < 0.9 && distToFrontBumper < 5) {
          emergencyBrake = true;
        } else if (distToFrontBumper < config.obstacleSlowdownDist) {
          const slowdownFactor = Math.max(0, (distToFrontBumper - config.emergencyBrakeDist) / (config.obstacleSlowdownDist - config.emergencyBrakeDist));
          const safeSpeed = Math.max(config.minSpeed * 0.5, config.baseSpeed * slowdownFactor * 0.7);
          targetSpeed = Math.min(targetSpeed, safeSpeed);
        }
      }
    }
  }
  let accelCmd;
  if (emergencyBrake) {
    accelCmd = -6;
    speedIntegral = 0;
  } else {
    const dt = 1 / 60;
    const speedError = targetSpeed - s.v;
    speedIntegral = clamp(speedIntegral + speedError * dt, -10, 10);
    const speedDerivative = (speedError - lastSpeedError) / dt;
    lastSpeedError = speedError;
    accelCmd = config.kpSpeed * speedError + config.kiSpeed * speedIntegral + config.kdSpeed * speedDerivative;
    accelCmd = clamp(accelCmd, -6, 3);
  }
  return {
    steer: clampedSteer,
    accel: accelCmd
  };
}

// src/sim/sim.test.ts
function assert(condition, message) {
  if (!condition) {
    throw new Error(`FAIL: ${message}`);
  }
  console.log(`  PASS: ${message}`);
}
function testCircleDrivingRadius() {
  console.log("\n--- Running Test 1: Circle Driving Radius ---");
  const dt = 1 / 60;
  const targetDelta = 0.25;
  const targetSpeed = 6;
  const expectedRadius = WHEELBASE / Math.tan(targetDelta);
  let state = {
    x: 0,
    y: 0,
    theta: 0,
    v: targetSpeed,
    delta: targetDelta
  };
  const cmd = {
    steer: targetDelta,
    accel: 0
  };
  const period = 2 * Math.PI * expectedRadius / targetSpeed;
  const steps = Math.ceil(period / dt);
  const expectedCenterX = -expectedRadius;
  const expectedCenterY = 0;
  let maxRadiusDeviation = 0;
  for (let i = 0; i < steps; i++) {
    state = stepVehicle(state, cmd, dt);
    const measuredDistToCenter = Math.hypot(
      state.x - expectedCenterX,
      state.y - expectedCenterY
    );
    const deviation = Math.abs(measuredDistToCenter - expectedRadius);
    if (deviation > maxRadiusDeviation) {
      maxRadiusDeviation = deviation;
    }
  }
  console.log(`  Theoretical Radius: ${expectedRadius.toFixed(4)} m`);
  console.log(`  Max deviation during full turn: ${maxRadiusDeviation.toFixed(4)} m`);
  assert(
    maxRadiusDeviation < 0.08,
    `Measured circle radius matches theoretical L/tan(delta) within 0.08 m (got deviation: ${maxRadiusDeviation.toFixed(4)} m)`
  );
}
function testSCurvePursuitTracking() {
  console.log("\n--- Running Test 2: Pursuit S-Curve Tracking ---");
  const dt = 1 / 60;
  const path = [];
  const pathSpacing = 0.5;
  const pathLength = 60;
  for (let y = 0; y <= pathLength; y += pathSpacing) {
    const normalizedY = y / pathLength;
    const x = 2.2 * Math.sin(2 * Math.PI * normalizedY) * Math.sin(Math.PI * normalizedY);
    path.push({ x, y });
  }
  let state = {
    x: 0,
    y: 0,
    theta: 0,
    v: 5,
    // Cruise speed
    delta: 0
  };
  let maxTrackingError = 0;
  let totalTrackingError = 0;
  let sampleCount = 0;
  for (let step = 0; step < 800; step++) {
    if (state.y >= 55) break;
    const cmd = pursuit(state, path);
    state = stepVehicle(state, cmd, dt);
    let minDistToPath = Infinity;
    for (let i = 0; i < path.length; i++) {
      const d = Math.hypot(state.x - path[i].x, state.y - path[i].y);
      if (d < minDistToPath) {
        minDistToPath = d;
      }
    }
    if (state.y > 1) {
      sampleCount++;
      totalTrackingError += minDistToPath;
      if (minDistToPath > maxTrackingError) {
        maxTrackingError = minDistToPath;
      }
    }
  }
  const avgTrackingError = sampleCount > 0 ? totalTrackingError / sampleCount : 0;
  console.log(`  Max Cross-Track Error: ${maxTrackingError.toFixed(4)} m`);
  console.log(`  Avg Cross-Track Error: ${avgTrackingError.toFixed(4)} m`);
  assert(
    maxTrackingError <= 0.3,
    `Car tracks S-curve path within 0.3 m requirement (got max error: ${maxTrackingError.toFixed(4)} m)`
  );
}
function runAllSimTests() {
  try {
    testCircleDrivingRadius();
    testSCurvePursuitTracking();
    console.log("\n[SUCCESS] ALL SIMULATION UNIT TESTS PASSED!\n");
    return true;
  } catch (err) {
    console.error("\n[FAILURE] Simulation unit test failed:", err);
    return false;
  }
}
runAllSimTests();
export {
  runAllSimTests,
  testCircleDrivingRadius,
  testSCurvePursuitTracking
};
