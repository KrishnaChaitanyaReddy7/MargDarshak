import { VehicleState } from '../core/types';

/**
 * Telemetry snapshot exported every simulation frame for HUD (Member D) and logs.
 */
export interface Telemetry {
  speed: number;             // m/s
  speedKmph: number;         // km/h (display convenience)
  steerAngle: number;        // rad
  steerAngleDeg: number;     // deg
  mode: 'manual' | 'autopilot';
  replansPerSec: number;     // replan frequency
  collisionCount: number;    // cumulative collisions
  isColliding: boolean;      // instantaneous collision flag
  minObstacleDist: number;   // distance in meters to closest obstacle
  scenario: string;          // current active scenario name
  paused: boolean;           // simulation pause state
  fps: number;               // frame rate
}

/**
 * Creates an empty/initial telemetry object
 */
export function createInitialTelemetry(scenarioName: string = 'Dense Mixed Traffic'): Telemetry {
  return {
    speed: 0,
    speedKmph: 0,
    steerAngle: 0,
    steerAngleDeg: 0,
    mode: 'autopilot',
    replansPerSec: 0,
    collisionCount: 0,
    isColliding: false,
    minObstacleDist: 999.0,
    scenario: scenarioName,
    paused: false,
    fps: 60,
  };
}

/**
 * Updates a telemetry object from current vehicle state and system status
 */
export function updateTelemetry(
  t: Telemetry,
  s: VehicleState,
  mode: 'manual' | 'autopilot',
  replansPerSec: number,
  collisionCount: number,
  isColliding: boolean,
  minObstacleDist: number,
  scenario: string,
  paused: boolean,
  fps: number
): Telemetry {
  t.speed = s.v;
  t.speedKmph = s.v * 3.6;
  t.steerAngle = s.delta;
  t.steerAngleDeg = (s.delta * 180) / Math.PI;
  t.mode = mode;
  t.replansPerSec = replansPerSec;
  t.collisionCount = collisionCount;
  t.isColliding = isColliding;
  t.minObstacleDist = minObstacleDist;
  t.scenario = scenario;
  t.paused = paused;
  t.fps = fps;
  return t;
}
