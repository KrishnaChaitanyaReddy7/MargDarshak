import { ControlCmd, Path, VehicleState } from '../core/types';
import { pursuit } from './controller';
import { stepVehicle, WHEELBASE } from './vehicle';

declare const process: any;

/**
 * Custom light-weight test runner for CI and standalone local verification
 */
function assert(condition: boolean, message: string): void {
  if (!condition) {
    throw new Error(`FAIL: ${message}`);
  }
  console.log(`  PASS: ${message}`);
}

/**
 * TEST 1: Circle Driving Radius
 * Requirement: Circle driving radius matches wheelbase / tan(delta)
 */
export function testCircleDrivingRadius(): void {
  console.log('\n--- Running Test 1: Circle Driving Radius ---');
  const dt = 1 / 60;
  const targetDelta = 0.25; // radians
  const targetSpeed = 6.0; // m/s
  const expectedRadius = WHEELBASE / Math.tan(targetDelta); // ~10.574 m

  // Initialize vehicle state
  let state: VehicleState = {
    x: 0,
    y: 0,
    theta: 0,
    v: targetSpeed,
    delta: targetDelta,
  };

  const cmd: ControlCmd = {
    steer: targetDelta,
    accel: 0,
  };

  // Run vehicle for one full revolution: T = 2 * PI * R / v
  const period = (2 * Math.PI * expectedRadius) / targetSpeed;
  const steps = Math.ceil(period / dt);

  // In circle starting at (0,0) facing +y with delta > 0 (CCW left turn):
  // The center of the circle is to the left (-x axis) at (-R, 0).
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

/**
 * TEST 2: Pursuit on S-Curve Path with No Obstacles
 * Requirement: Car must track within 0.3 m
 */
export function testSCurvePursuitTracking(): void {
  console.log('\n--- Running Test 2: Pursuit S-Curve Tracking ---');
  const dt = 1 / 60;

  // 1. Generate hardcoded smooth S-curve path (ISO 3888 standard double lane change curve)
  // Starts tangentially at origin (0,0) facing +y, smoothly bends right, then bends left, and straightens
  const path: Path = [];
  const pathSpacing = 0.5;
  const pathLength = 60.0;

  for (let y = 0; y <= pathLength; y += pathSpacing) {
    // S-curve with smooth tangential entry (dx/dy = 0 at y = 0)
    const normalizedY = y / pathLength;
    const x = 2.2 * Math.sin(2 * Math.PI * normalizedY) * Math.sin(Math.PI * normalizedY);
    path.push({ x, y });
  }

  // 2. Start vehicle at path origin aligned with path tangent
  let state: VehicleState = {
    x: 0,
    y: 0,
    theta: 0,
    v: 5.0, // Cruise speed
    delta: 0,
  };

  let maxTrackingError = 0;
  let totalTrackingError = 0;
  let sampleCount = 0;

  // Simulate tracking for 800 steps (~13 seconds)
  for (let step = 0; step < 800; step++) {
    if (state.y >= 55.0) break;

    const cmd = pursuit(state, path);
    state = stepVehicle(state, cmd, dt);

    // Compute cross-track error to nearest path segment
    let minDistToPath = Infinity;
    for (let i = 0; i < path.length; i++) {
      const d = Math.hypot(state.x - path[i].x, state.y - path[i].y);
      if (d < minDistToPath) {
        minDistToPath = d;
      }
    }

    // Evaluate tracking error after first meter of motion
    if (state.y > 1.0) {
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

/**
 * Runner function for test suite
 */
export function runAllSimTests(): boolean {
  try {
    testCircleDrivingRadius();
    testSCurvePursuitTracking();
    console.log('\n[SUCCESS] ALL SIMULATION UNIT TESTS PASSED!\n');
    return true;
  } catch (err) {
    console.error('\n[FAILURE] Simulation unit test failed:', err);
    return false;
  }
}

// Auto-execute when run directly
runAllSimTests();

