import { ControlCmd, VehicleState } from '../core/types';

/**
 * Kinematic Bicycle Model Constants
 */
export const WHEELBASE = 2.7; // meters
export const STEER_LIMIT = 0.6; // radians (clamp ±0.6 rad ~ ±34.38 deg)
export const STEER_RATE_LIMIT = 0.8; // radians per second
export const ACCEL_MIN = -6.0; // m/s² (maximum braking)
export const ACCEL_MAX = 3.0; // m/s² (maximum throttle)

/**
 * Clamp helper function
 */
export function clamp(val: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, val));
}

/**
 * Normalizes an angle into the [-PI, PI] range
 */
export function normalizeAngle(angle: number): number {
  let a = angle % (2 * Math.PI);
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

/**
 * Steps the vehicle state forward in time by dt seconds using
 * a kinematic bicycle model with semi-implicit Euler integration.
 *
 * Coordinate Convention:
 * - World frame: x = right, y = forward (up on canvas).
 * - Heading theta = 0 facing +y, positive = counter-clockwise (CCW).
 * - Steering delta > 0 turns car CCW (left).
 *
 * @param s Current vehicle state
 * @param c Commanded control inputs (steer, accel)
 * @param dt Timestep in seconds (e.g. 1/60)
 * @returns Updated vehicle state
 */
export function stepVehicle(s: VehicleState, c: ControlCmd, dt: number): VehicleState {
  // 1. Clamp commanded steering and acceleration to vehicle physical limits
  const targetSteer = clamp(c.steer, -STEER_LIMIT, STEER_LIMIT);
  const targetAccel = clamp(c.accel, ACCEL_MIN, ACCEL_MAX);

  // 2. Apply steering rate limit (max 0.8 rad/s)
  const maxSteerDelta = STEER_RATE_LIMIT * dt;
  const steerDiff = targetSteer - s.delta;
  const deltaChange = clamp(steerDiff, -maxSteerDelta, maxSteerDelta);
  const nextDelta = clamp(s.delta + deltaChange, -STEER_LIMIT, STEER_LIMIT);

  // 3. Update velocity (no reverse: clamp to >= 0)
  const nextV = Math.max(0, s.v + targetAccel * dt);

  // 4. Semi-implicit Euler integration:
  // Calculate yaw rate (theta_dot) using the updated velocity and steering angle
  const omega = (nextV / WHEELBASE) * Math.tan(nextDelta);
  const nextTheta = normalizeAngle(s.theta + omega * dt);

  // 5. Update positions using updated heading and updated velocity
  // With theta = 0 facing +y and CCW positive:
  // dx/dt = -v * sin(theta)
  // dy/dt =  v * cos(theta)
  const nextX = s.x - nextV * Math.sin(nextTheta) * dt;
  const nextY = s.y + nextV * Math.cos(nextTheta) * dt;

  return {
    x: nextX,
    y: nextY,
    theta: nextTheta,
    v: nextV,
    delta: nextDelta,
  };
}
