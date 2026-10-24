import { ControlCmd, Obstacle, Path, VehicleState } from '../core/types';
import { clamp, WHEELBASE } from './vehicle';

/**
 * Controller configuration and tuning parameters
 */
export interface ControllerConfig {
  baseSpeed: number; // m/s (~8 m/s = 28.8 km/h)
  minSpeed: number; // m/s
  lookaheadBase: number; // meters (3 m)
  lookaheadGain: number; // s (0.4 * v)
  minLookahead: number; // meters
  maxLookahead: number; // meters
  kpSpeed: number;
  kiSpeed: number;
  kdSpeed: number;
  emergencyBrakeDist: number; // meters (3 m corridor)
  obstacleSlowdownDist: number; // meters (8 m ahead)
  egoLength: number; // meters (4.2 m)
  egoWidth: number; // meters (1.8 m)
}

export const DEFAULT_CONTROLLER_CONFIG: ControllerConfig = {
  baseSpeed: 8.0,
  minSpeed: 1.5,
  lookaheadBase: 3.0,
  lookaheadGain: 0.4,
  minLookahead: 3.0,
  maxLookahead: 18.0,
  kpSpeed: 2.0,
  kiSpeed: 0.05,
  kdSpeed: 0.2,
  emergencyBrakeDist: 3.0,
  obstacleSlowdownDist: 8.0,
  egoLength: 4.2,
  egoWidth: 1.8,
};

// Internal PID speed state
let speedIntegral = 0;
let lastSpeedError = 0;

/**
 * Resets the controller internal integral and derivative state
 */
export function resetControllerState(): void {
  speedIntegral = 0;
  lastSpeedError = 0;
}

/**
 * Transforms a world point into the vehicle's local frame.
 * Vehicle convention:
 * - x = right, y = forward, theta = 0 facing +y, CCW positive.
 * Local frame:
 * - localX > 0 is to the right, localX < 0 is to the left.
 * - localY > 0 is forward ahead of the vehicle.
 */
export function worldToVehicleLocal(
  wx: number,
  wy: number,
  s: VehicleState
): { localX: number; localY: number } {
  const dx = wx - s.x;
  const dy = wy - s.y;
  const cosT = Math.cos(s.theta);
  const sinT = Math.sin(s.theta);

  // Forward unit vector: (-sinT, cosT)
  // Right unit vector:   (cosT, sinT)
  const localY = -dx * sinT + dy * cosT;
  const localX = dx * cosT + dy * sinT;

  return { localX, localY };
}

/**
 * Finds the lookahead target point on the path at lookahead distance L.
 * Uses sequential search starting from the closest waypoint to the vehicle,
 * and interpolates between waypoints to guarantee continuous, jitter-free tracking.
 */
export function findLookaheadPoint(
  s: VehicleState,
  path: Path,
  lookaheadDist: number
): { localX: number; localY: number; dist: number } {
  if (path.length === 0) {
    return { localX: 0, localY: lookaheadDist, dist: lookaheadDist };
  }

  // 1. Find the index of the closest waypoint to the vehicle
  let closestIdx = 0;
  let minCenterDist = Infinity;
  for (let i = 0; i < path.length; i++) {
    const d = Math.hypot(path[i].x - s.x, path[i].y - s.y);
    if (d < minCenterDist) {
      minCenterDist = d;
      closestIdx = i;
    }
  }

  // 2. Search forward along the path for the segment bracketing lookaheadDist
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
    // If beyond lookahead distance, pick the last waypoint
    const last = path[path.length - 1];
    targetWorldX = last.x;
    targetWorldY = last.y;
  }

  const { localX, localY } = worldToVehicleLocal(targetWorldX, targetWorldY, s);
  const actualDist = Math.hypot(localX, localY);

  return { localX, localY, dist: Math.max(0.5, actualDist) };
}

/**
 * Pure pursuit steering with speed-dependent lookahead (L = 3 + 0.4*v),
 * plus PID speed control with curvature slowdown and obstacle safety corridors.
 *
 * Exact Contract Signature:
 * pursuit(s: VehicleState, path: Path): ControlCmd
 * (Optional obstacles parameter provided for obstacle-aware speed control)
 *
 * @param s Current vehicle state
 * @param path Ordered waypoints from ego to goal (~0.5 m spacing)
 * @param obstacles Optional list of dynamic/static obstacles
 * @param config Optional controller tuning configuration
 */
export function pursuit(
  s: VehicleState,
  path: Path,
  obstacles: Obstacle[] = [],
  config: ControllerConfig = DEFAULT_CONTROLLER_CONFIG
): ControlCmd {
  // 1. Calculate speed-dependent lookahead distance: L = 3 + 0.4 * v
  const L = clamp(
    config.lookaheadBase + config.lookaheadGain * s.v,
    config.minLookahead,
    config.maxLookahead
  );

  // 2. Find target lookahead point on path in local frame
  const target = findLookaheadPoint(s, path, L);
  const actualL = Math.max(config.minLookahead, target.dist);

  // 3. Pure Pursuit steering calculation:
  // Turning left (localX < 0) requires positive steering delta.
  // Curvature: kappa = 2 * (-localX) / L^2
  // Steer: delta = atan(L_wheelbase * kappa)
  const steerCmd = Math.atan2(2 * WHEELBASE * -target.localX, actualL * actualL);
  const clampedSteer = clamp(steerCmd, -0.6, 0.6);

  // 4. Path Curvature assessment for speed regulation
  const pathCurvature = Math.abs(2 * -target.localX / (actualL * actualL));
  let targetSpeed = config.baseSpeed / (1.0 + 3.5 * pathCurvature);
  targetSpeed = Math.max(config.minSpeed, targetSpeed);

  // 5. Obstacle Safety Evaluation
  let emergencyBrake = false;
  let minObstacleDistanceAhead = Infinity;

  const egoHalfLen = config.egoLength / 2;
  const egoHalfWidth = config.egoWidth / 2;

  for (const obs of obstacles) {
    const { localX, localY } = worldToVehicleLocal(obs.x, obs.y, s);
    const obsHalfLen = obs.l / 2;
    const obsHalfWidth = obs.w / 2;

    // Corridor half-width includes ego width + obstacle width + margin
    const corridorHalfWidth = egoHalfWidth + obsHalfWidth + 0.35;

    // Check if obstacle is inside forward corridor
    if (Math.abs(localX) <= corridorHalfWidth) {
      const distToFrontBumper = localY - egoHalfLen - obsHalfLen;

      if (distToFrontBumper > -0.5) {
        if (distToFrontBumper < minObstacleDistanceAhead) {
          minObstacleDistanceAhead = distToFrontBumper;
        }

        // Emergency brake condition: within 3 m time-to-collision / distance corridor
        const ttc = distToFrontBumper / Math.max(0.2, s.v);
        if (distToFrontBumper <= config.emergencyBrakeDist || (ttc < 0.9 && distToFrontBumper < 5.0)) {
          emergencyBrake = true;
        } else if (distToFrontBumper < config.obstacleSlowdownDist) {
          // Slow down automatically when any obstacle is within 8 m ahead
          const slowdownFactor = Math.max(0, (distToFrontBumper - config.emergencyBrakeDist) / (config.obstacleSlowdownDist - config.emergencyBrakeDist));
          const safeSpeed = Math.max(config.minSpeed * 0.5, config.baseSpeed * slowdownFactor * 0.7);
          targetSpeed = Math.min(targetSpeed, safeSpeed);
        }
      }
    }
  }

  // 6. Acceleration Command via PID
  let accelCmd: number;
  if (emergencyBrake) {
    accelCmd = -6.0; // Maximum emergency deceleration clamp
    speedIntegral = 0; // Reset integrator on emergency
  } else {
    const dt = 1 / 60; // Fixed nominal timestep
    const speedError = targetSpeed - s.v;
    speedIntegral = clamp(speedIntegral + speedError * dt, -10, 10);
    const speedDerivative = (speedError - lastSpeedError) / dt;
    lastSpeedError = speedError;

    accelCmd =
      config.kpSpeed * speedError +
      config.kiSpeed * speedIntegral +
      config.kdSpeed * speedDerivative;

    accelCmd = clamp(accelCmd, -6.0, 3.0);
  }

  return {
    steer: clampedSteer,
    accel: accelCmd,
  };
}
