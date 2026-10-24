import { ControlCmd, Path, VehicleState, World } from './core/types';
import { EGO_LENGTH, EGO_WIDTH, evaluateCollisions } from './sim/collision';
import { pursuit, resetControllerState } from './sim/controller';
import { Telemetry, updateTelemetry } from './sim/telemetry';
import { clamp, stepVehicle } from './sim/vehicle';
import { plan } from './planning/planner';
import { createWorld, stepWorld } from './planning/world';
import { render2D } from './ui/hud';
import { mount as mount3D, render3D } from './view3d/scene3d';

// Simulation Configuration
const DT = 1 / 60; // 60 Hz fixed timestep (~0.01667 s)
const REPLAN_INTERVAL_MS = 100; // 10 Hz replanning frequency in Autopilot

// Simulation State
let mode: 'autopilot' | 'manual' = 'autopilot';
let isPaused = false;
let currentScenario = '1';
let collisionCount = 0;
let wasCollidingLastFrame = false;

// Initial Vehicle State at origin
const initialVehicleState: VehicleState = {
  x: 0,
  y: 0,
  theta: 0, // facing +y (forward)
  v: 0,
  delta: 0,
};

let vehicleState: VehicleState = { ...initialVehicleState };
let world: World = createWorld(currentScenario);
let plannedPath: Path = [];

// Replan & Telemetry Performance Tracking
let lastReplanTime = 0;
let replanCounter = 0;
let replansPerSec = 0;
let replanFpsTimer = performance.now();
let frameCount = 0;
let lastFpsTime = performance.now();
let currentFps = 60;

// Manual input state (smooth control)
const keysPressed: Record<string, boolean> = {};
let manualSteerTarget = 0.0;
let manualAccelTarget = 0.0;

// Global exported telemetry object
export const currentTelemetry: Telemetry = {
  speed: 0,
  speedKmph: 0,
  steerAngle: 0,
  steerAngleDeg: 0,
  mode: 'autopilot',
  replansPerSec: 0,
  collisionCount: 0,
  isColliding: false,
  minObstacleDist: 999.0,
  scenario: 'Dense Mixed Traffic',
  paused: false,
  fps: 60,
};

/**
 * Executes a path replanning pass immediately
 */
export function replanNow(): void {
  plannedPath = plan(world, vehicleState);
  replanCounter++;
}

/**
 * Resets the simulation to starting state
 */
export function resetSimulation(): void {
  vehicleState = { ...initialVehicleState };
  world = createWorld(currentScenario);
  resetControllerState();
  replanNow();
  manualSteerTarget = 0.0;
  manualAccelTarget = 0.0;
  collisionCount = 0;
  wasCollidingLastFrame = false;
}

/**
 * Switches the active scenario (1: Dense Mixed, 2: Bottleneck, 3: Moving Traffic)
 */
export function switchScenario(scenarioKey: string): void {
  currentScenario = scenarioKey;
  resetSimulation();
}

/**
 * Keyboard input listeners
 */
function setupInputListeners(): void {
  window.addEventListener('keydown', (e) => {
    keysPressed[e.key] = true;
    keysPressed[e.code] = true;

    // Mode Toggle (M)
    if (e.key === 'm' || e.key === 'M') {
      mode = mode === 'autopilot' ? 'manual' : 'autopilot';
      if (mode === 'autopilot') {
        replanNow();
      }
    }

    // Pause Toggle (Space)
    if (e.code === 'Space') {
      isPaused = !isPaused;
      e.preventDefault();
    }

    // Reset (R)
    if (e.key === 'r' || e.key === 'R') {
      resetSimulation();
    }

    // Switch Scenario (1, 2, 3)
    if (e.key === '1') switchScenario('1');
    if (e.key === '2') switchScenario('2');
    if (e.key === '3') switchScenario('3');

    // Interactive obstacle addition/movement (triggers immediate replan)
    if (e.key === 'o' || e.key === 'O') {
      // Add a dynamic obstacle 15m ahead of vehicle to test reactive avoidance
      world.obstacles.push({
        id: `dynamic-${Date.now()}`,
        cls: 'cow',
        x: vehicleState.x + (Math.random() > 0.5 ? 0.8 : -0.8),
        y: vehicleState.y + 14.0 + Math.random() * 5.0,
        w: 1.2,
        l: 2.2,
        heading: 0,
        vx: 0,
        vy: 0,
      });
      // Replan immediately whenever an obstacle is added/moved
      replanNow();
    }
  });

  window.addEventListener('keyup', (e) => {
    keysPressed[e.key] = false;
    keysPressed[e.code] = false;
  });
}

/**
 * Computes smooth manual driving commands based on active keyboard inputs
 */
function computeManualCommand(dt: number): ControlCmd {
  const isUp = keysPressed['ArrowUp'] || keysPressed['KeyW'];
  const isDown = keysPressed['ArrowDown'] || keysPressed['KeyS'];
  const isLeft = keysPressed['ArrowLeft'] || keysPressed['KeyA'];
  const isRight = keysPressed['ArrowRight'] || keysPressed['KeyD'];

  // Smooth steering: ramp towards ±0.6 rad at steering rate limit
  const maxSteer = 0.6;
  const steerRate = 1.2; // rad/s input ramp rate

  if (isLeft && !isRight) {
    // Steer left (positive angle in CCW convention)
    manualSteerTarget = Math.min(maxSteer, manualSteerTarget + steerRate * dt);
  } else if (isRight && !isLeft) {
    // Steer right (negative angle in CCW convention)
    manualSteerTarget = Math.max(-maxSteer, manualSteerTarget - steerRate * dt);
  } else {
    // Self-centering spring behavior
    if (manualSteerTarget > 0) {
      manualSteerTarget = Math.max(0, manualSteerTarget - steerRate * 1.5 * dt);
    } else if (manualSteerTarget < 0) {
      manualSteerTarget = Math.min(0, manualSteerTarget + steerRate * 1.5 * dt);
    }
  }

  // Smooth throttle and brake
  if (isUp && !isDown) {
    manualAccelTarget = 2.5; // Throttle acceleration (m/s²)
  } else if (isDown && !isUp) {
    manualAccelTarget = -5.5; // Heavy braking (m/s²)
  } else {
    // Coasting drag deceleration
    manualAccelTarget = vehicleState.v > 0.1 ? -1.0 : 0.0;
  }

  return {
    steer: clamp(manualSteerTarget, -0.6, 0.6),
    accel: clamp(manualAccelTarget, -6.0, 3.0),
  };
}

/**
 * Fixed Timestep Simulation Tick (60 Hz)
 */
function tickSimulation(): void {
  if (isPaused) return;

  const now = performance.now();

  // 1. Step the dynamic world (moves obstacles, traffic, cows, etc.)
  stepWorld(world, DT);

  // 2. Control Command Determination
  let cmd: ControlCmd;

  if (mode === 'manual') {
    cmd = computeManualCommand(DT);
  } else {
    // Autopilot Mode:
    // Replan at 10 Hz (every 100 ms)
    if (now - lastReplanTime >= REPLAN_INTERVAL_MS || plannedPath.length === 0) {
      replanNow();
      lastReplanTime = now;
    }

    // Pure Pursuit controller + PID speed control
    cmd = pursuit(vehicleState, plannedPath, world.obstacles);
  }

  // 3. Step Vehicle Kinematic Bicycle Model
  vehicleState = stepVehicle(vehicleState, cmd, DT);

  // 4. Collision Detection using Oriented Bounding Boxes (SAT)
  const col = evaluateCollisions(vehicleState, world.obstacles, EGO_WIDTH, EGO_LENGTH);

  // Increment collision count only on new collision entry
  if (col.isColliding && !wasCollidingLastFrame) {
    collisionCount++;
  }
  wasCollidingLastFrame = col.isColliding;

  // 5. Update Telemetry Object
  updateTelemetry(
    currentTelemetry,
    vehicleState,
    mode,
    replansPerSec,
    collisionCount,
    col.isColliding,
    col.minObstacleDistance,
    currentScenario === '1'
      ? 'Dense Mixed Traffic'
      : currentScenario === '2'
      ? 'Narrow Bottleneck'
      : 'Moving Traffic',
    isPaused,
    currentFps
  );
}

/**
 * Main Animation Loop with Fixed Timestep Accumulator
 */
let accumulator = 0;
let lastFrameTime = performance.now();

function mainLoop(canvas2dCtx: CanvasRenderingContext2D, has3D: boolean): void {
  const now = performance.now();
  let frameDelta = (now - lastFrameTime) / 1000;
  lastFrameTime = now;

  // Cap delta to prevent spiral of death during backgrounding
  if (frameDelta > 0.1) frameDelta = 0.1;
  accumulator += frameDelta;

  // Measure FPS and Replan Frequency
  frameCount++;
  if (now - lastFpsTime >= 1000) {
    currentFps = frameCount;
    frameCount = 0;
    lastFpsTime = now;
  }
  if (now - replanFpsTimer >= 1000) {
    replansPerSec = replanCounter;
    replanCounter = 0;
    replanFpsTimer = now;
  }

  // Consume fixed physics steps
  while (accumulator >= DT) {
    tickSimulation();
    accumulator -= DT;
  }

  // Render 2D Canvas HUD
  render2D(canvas2dCtx, vehicleState, world, plannedPath);

  // Render 3D Three.js if enabled
  if (has3D) {
    render3D(vehicleState, world, plannedPath);
  }

  requestAnimationFrame(() => mainLoop(canvas2dCtx, has3D));
}

/**
 * Initializes the full simulation application
 */
export function initApp(): void {
  setupInputListeners();
  replanNow();

  const canvas2d = document.getElementById('canvas2d') as HTMLCanvasElement;
  if (!canvas2d) {
    console.error('Canvas #canvas2d not found in DOM.');
    return;
  }

  const ctx2d = canvas2d.getContext('2d');
  if (!ctx2d) {
    console.error('Failed to get 2D context.');
    return;
  }

  // Handle high-DPI canvas resizing
  function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas2d.getBoundingClientRect();
    canvas2d.width = rect.width * dpr;
    canvas2d.height = rect.height * dpr;
    ctx2d!.scale(dpr, dpr);
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // Check for 3D container
  const container3D = document.getElementById('scene3d');
  let has3D = false;
  if (container3D) {
    try {
      mount3D(container3D);
      has3D = true;
    } catch {
      has3D = false;
    }
  }

  // Start the simulation loop
  requestAnimationFrame(() => mainLoop(ctx2d, has3D));
}

// Auto-boot if running in browser window
if (typeof window !== 'undefined') {
  window.addEventListener('DOMContentLoaded', () => {
    initApp();
  });
}
