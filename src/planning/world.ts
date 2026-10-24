import type { Vec2, Obstacle, ObstacleClass, World } from '../core/types';

/**
 * Seeded pseudo-random number generator (Mulberry32) for deterministic simulations.
 */
export class SeededRNG {
  private s: number;

  constructor(seed: number = 42) {
    this.s = seed >>> 0;
  }

  /** Returns uniform random number in [0, 1) */
  next(): number {
    let t = (this.s += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  /** Uniform random float in [min, max) */
  range(min: number, max: number): number {
    return min + this.next() * (max - min);
  }

  /** Random integer in [min, max] */
  rangeInt(min: number, max: number): number {
    return Math.floor(this.range(min, max + 1));
  }
}

/**
 * Internal state for obstacle behaviors (weave, zigzag, crossing, cow pauses).
 */
interface ObstacleBehaviorState {
  // Auto-rickshaw weaving
  weavePhase?: number;
  weaveFreq?: number;
  weaveAmp?: number;
  baseVy?: number;

  // Two-wheeler zigzag
  zigzagTimer?: number;
  zigzagDir?: number;
  targetVx?: number;

  // Pedestrian crossing
  crossDir?: number;
  crossSpeed?: number;
  pauseTimer?: number;

  // Cow standing/moving
  isStopped?: boolean;
  timer?: number; // 5-10s stopped, 2-4s moving
  moveSpeed?: number;
  moveHeading?: number;

  // General bounds
  minX?: number;
  maxX?: number;
}

// Map associating obstacle id with its dynamic behavior state
const behaviorMap = new Map<string, ObstacleBehaviorState>();
let nextObstacleId = 1;

/**
 * Reset or initialize obstacle behavior state.
 */
function initObstacleBehavior(obs: Obstacle, rng: SeededRNG, minX: number = -3.5, maxX: number = 3.5): void {
  const state: ObstacleBehaviorState = { minX, maxX };

  switch (obs.cls) {
    case 'auto':
      state.weavePhase = rng.range(0, Math.PI * 2);
      state.weaveFreq = rng.range(0.8, 1.4); // rad/s
      state.weaveAmp = rng.range(0.8, 1.4); // m/s lateral
      state.baseVy = obs.vy !== 0 ? obs.vy : rng.range(4.0, 6.0);
      obs.vy = state.baseVy;
      break;

    case 'two_wheeler':
      state.zigzagTimer = rng.range(1.0, 2.5);
      state.zigzagDir = rng.next() > 0.5 ? 1 : -1;
      state.targetVx = state.zigzagDir * rng.range(1.5, 2.8);
      state.baseVy = obs.vy !== 0 ? obs.vy : rng.range(7.0, 9.5);
      obs.vy = state.baseVy;
      break;

    case 'pedestrian':
      state.crossDir = obs.vx !== 0 ? Math.sign(obs.vx) : (rng.next() > 0.5 ? 1 : -1);
      state.crossSpeed = rng.range(0.9, 1.4);
      state.pauseTimer = 0;
      obs.vx = state.crossDir * state.crossSpeed;
      obs.vy = rng.range(-0.1, 0.2);
      break;

    case 'cow':
      state.isStopped = true;
      state.timer = rng.range(5.0, 10.0); // Stop in middle of road for 5-10 s
      state.moveSpeed = 0;
      state.moveHeading = obs.heading;
      obs.vx = 0;
      obs.vy = 0;
      break;

    case 'debris':
      obs.vx = 0;
      obs.vy = 0;
      break;

    default:
      // car / truck
      state.baseVy = obs.vy !== 0 ? obs.vy : (obs.cls === 'truck' ? 4.5 : 7.0);
      obs.vy = state.baseVy;
      break;
  }

  behaviorMap.set(obs.id, state);
}

/**
 * Standard dimensions for obstacle classes.
 */
export const OBSTACLE_DEFAULTS: Record<ObstacleClass, { w: number; l: number; vy: number }> = {
  auto: { w: 1.4, l: 2.6, vy: 5.0 },
  two_wheeler: { w: 0.7, l: 1.9, vy: 8.0 },
  pedestrian: { w: 0.5, l: 0.5, vy: 0.0 },
  cow: { w: 0.8, l: 2.0, vy: 0.0 },
  debris: { w: 1.0, l: 1.0, vy: 0.0 },
  car: { w: 1.8, l: 4.2, vy: 6.5 },
  truck: { w: 2.4, l: 7.0, vy: 4.5 },
};

/**
 * Adds an obstacle dynamically to the world.
 * Can be called by UI click handlers or scenarios.
 */
export function addObstacle(w: World, cls: ObstacleClass, x: number, y: number): Obstacle {
  const defaults = OBSTACLE_DEFAULTS[cls] || { w: 1.0, l: 1.0, vy: 0 };
  const id = `obs_${nextObstacleId++}_${cls}`;
  
  // Heading: cows stand angled or across the road; vehicles face forward (+y = 0 rad)
  let heading = 0;
  if (cls === 'cow') {
    heading = Math.PI / 2; // standing across the road
  }

  const obstacle: Obstacle = {
    id,
    cls,
    x,
    y,
    w: defaults.w,
    l: defaults.l,
    heading,
    vx: 0,
    vy: defaults.vy,
  };

  const rng = new SeededRNG(Math.abs((x * 1000 + y * 100) | 0) + 12345);
  initObstacleBehavior(obstacle, rng);

  w.obstacles.push(obstacle);
  return obstacle;
}

/**
 * Generates ragged shoulder noise for unmarked road edges.
 */
function shoulderNoise(y: number, seed: number): number {
  return (
    0.35 * Math.sin(y * 0.37 + seed) +
    0.20 * Math.sin(y * 0.83 + seed * 1.7) +
    0.15 * Math.cos(y * 1.49 + seed * 2.3)
  );
}

/**
 * Creates an unmarked road polygon (outer boundary of drivable road surface).
 * Coordinates: x = right, y = forward.
 */
function createRoadPolygon(
  centerlineFn: (y: number) => number,
  widthFn: (y: number) => number,
  yMin: number = -20,
  yMax: number = 220,
  yStep: number = 1.0,
  seed: number = 42
): Vec2[] {
  const leftPoints: Vec2[] = [];
  const rightPoints: Vec2[] = [];

  for (let y = yMin; y <= yMax; y += yStep) {
    const cx = centerlineFn(y);
    const halfW = widthFn(y) / 2;
    const leftNoise = shoulderNoise(y, seed);
    const rightNoise = shoulderNoise(y, seed + 100);

    leftPoints.push({ x: cx - halfW + leftNoise, y });
    rightPoints.push({ x: cx + halfW + rightNoise, y });
  }

  // Construct closed polygon: left going forward, right returning backward
  const polygon: Vec2[] = [];
  for (let i = 0; i < leftPoints.length; i++) {
    polygon.push(leftPoints[i]);
  }
  for (let i = rightPoints.length - 1; i >= 0; i--) {
    polygon.push(rightPoints[i]);
  }
  // Close loop
  polygon.push({ ...leftPoints[0] });

  return polygon;
}

/**
 * Creates the simulation world for a given scenario:
 * 1. "straight": 8 m wide road with ragged dirt shoulders (noisy edges)
 * 2. "curve": gentle S-bend with varying width 6-10 m
 * 3. "chaos": wide road, dense mixed traffic
 */
export function createWorld(scenario: string): World {
  behaviorMap.clear();
  nextObstacleId = 1;

  let seed = 1001;
  let roadPolygon: Vec2[];
  const obstacles: Obstacle[] = [];

  if (scenario === 'curve') {
    seed = 2002;
    const rng = new SeededRNG(seed);

    // S-bend centerline and varying width 6 - 10 m
    const centerline = (y: number) => 4.5 * Math.sin(y * 0.04) + 1.5 * Math.sin(y * 0.08);
    const width = (y: number) => 8.0 + 2.0 * Math.sin(y * 0.035 + 0.8);

    roadPolygon = createRoadPolygon(centerline, width, -20, 220, 1.0, seed);

    const world: World = {
      roadPolygons: [roadPolygon],
      obstacles,
      t: 0,
    };

    // Populate curve scenario with mixed traffic
    addObstacle(world, 'auto', centerline(22) - 1.2, 22);
    addObstacle(world, 'two_wheeler', centerline(38) + 1.5, 38);
    addObstacle(world, 'cow', centerline(55) + 0.2, 55);
    addObstacle(world, 'pedestrian', centerline(18) - 2.8, 18);
    addObstacle(world, 'debris', centerline(45) - 3.0, 45);
    addObstacle(world, 'car', centerline(75) - 0.5, 75);
    addObstacle(world, 'two_wheeler', centerline(90) - 1.0, 90);
    addObstacle(world, 'auto', centerline(110) + 1.2, 110);

    return world;
  } else if (scenario === 'chaos') {
    seed = 3003;
    const rng = new SeededRNG(seed);

    // Wide road (11-12m) with dense mixed traffic
    const centerline = (y: number) => 1.8 * Math.sin(y * 0.03);
    const width = (y: number) => 11.0 + 1.0 * Math.sin(y * 0.05);

    roadPolygon = createRoadPolygon(centerline, width, -20, 220, 1.0, seed);

    const world: World = {
      roadPolygons: [roadPolygon],
      obstacles,
      t: 0,
    };

    // Dense Indian traffic mix: rickshaws, bikes, cows, pedestrians, trucks, debris
    addObstacle(world, 'cow', 0.2, 20);
    addObstacle(world, 'auto', -2.2, 16);
    addObstacle(world, 'two_wheeler', 2.8, 25);
    addObstacle(world, 'pedestrian', -3.8, 12);
    addObstacle(world, 'debris', -4.2, 32);
    addObstacle(world, 'truck', 1.8, 40);
    addObstacle(world, 'two_wheeler', -1.5, 48);
    addObstacle(world, 'auto', 3.0, 52);
    addObstacle(world, 'cow', -1.0, 68);
    addObstacle(world, 'pedestrian', 4.0, 30);
    addObstacle(world, 'car', -2.5, 80);
    addObstacle(world, 'two_wheeler', 1.0, 85);
    addObstacle(world, 'debris', 3.5, 65);
    addObstacle(world, 'auto', -0.5, 100);

    return world;
  } else {
    // "straight" default scenario: 8 m wide road with ragged dirt shoulders
    seed = 1001;
    const rng = new SeededRNG(seed);

    const centerline = (_y: number) => 0;
    const width = (_y: number) => 8.0;

    roadPolygon = createRoadPolygon(centerline, width, -20, 220, 1.0, seed);

    const world: World = {
      roadPolygons: [roadPolygon],
      obstacles,
      t: 0,
    };

    // Representative obstacles on straight road
    addObstacle(world, 'auto', 1.2, 28);
    addObstacle(world, 'two_wheeler', -1.5, 45);
    addObstacle(world, 'pedestrian', -3.2, 14);
    addObstacle(world, 'debris', -3.0, 22);
    addObstacle(world, 'cow', 0.5, 60);
    addObstacle(world, 'car', -1.0, 80);

    return world;
  }
}

/**
 * Updates dynamic obstacles according to their behavioral rules for timestep dt.
 * Uses deterministic pseudo-random logic so demos are 100% reproducible.
 */
export function stepWorld(w: World, dt: number): void {
  w.t += dt;
  const rng = new SeededRNG(Math.floor(w.t * 60) + 777);

  for (const obs of w.obstacles) {
    let state = behaviorMap.get(obs.id);
    if (!state) {
      initObstacleBehavior(obs, rng);
      state = behaviorMap.get(obs.id)!;
    }

    switch (obs.cls) {
      case 'auto': {
        // Auto-rickshaw weaves side-to-side
        state.weavePhase = (state.weavePhase || 0) + dt * (state.weaveFreq || 1.0);
        const desiredVx = (state.weaveAmp || 1.0) * Math.cos(state.weavePhase);
        obs.vx = desiredVx;
        obs.vy = state.baseVy || 5.0;

        // Turn heading towards velocity vector (theta = 0 facing +y, positive CCW)
        obs.heading = Math.atan2(-obs.vx, obs.vy);
        break;
      }

      case 'two_wheeler': {
        // Fast, rapid zigzag cuts
        state.zigzagTimer = (state.zigzagTimer || 1.5) - dt;
        if (state.zigzagTimer <= 0) {
          state.zigzagDir = -(state.zigzagDir || 1);
          state.zigzagTimer = rng.range(1.2, 2.5);
          state.targetVx = (state.zigzagDir || 1) * rng.range(1.5, 2.8);
        }
        const targetVx = state.targetVx || 0;
        obs.vx += (targetVx - obs.vx) * Math.min(1.0, dt * 5.0);
        obs.vy = state.baseVy || 8.0;
        obs.heading = Math.atan2(-obs.vx, obs.vy);
        break;
      }

      case 'pedestrian': {
        // Pedestrians cross the road laterally, occasionally pausing
        if ((state.pauseTimer || 0) > 0) {
          state.pauseTimer = (state.pauseTimer || 0) - dt;
          obs.vx = 0;
          obs.vy = 0;
        } else {
          obs.vx = (state.crossDir || 1) * (state.crossSpeed || 1.1);
          obs.vy = 0.15; // slight forward drift

          // If reached road edge, pause and reverse direction
          if (obs.x > 3.6 && (state.crossDir || 1) > 0) {
            state.crossDir = -1;
            state.pauseTimer = rng.range(1.0, 2.5);
          } else if (obs.x < -3.6 && (state.crossDir || 1) < 0) {
            state.crossDir = 1;
            state.pauseTimer = rng.range(1.0, 2.5);
          }
          obs.heading = Math.atan2(-obs.vx, obs.vy);
        }
        break;
      }

      case 'cow': {
        // Cows stop in the middle of the road for 5-10 s, then move slowly
        state.timer = (state.timer || 7.0) - dt;
        if (state.timer <= 0) {
          if (state.isStopped) {
            // Start moving slowly for 2-4 seconds
            state.isStopped = false;
            state.timer = rng.range(2.0, 4.0);
            state.moveSpeed = rng.range(0.3, 0.6);
            state.moveHeading = rng.range(-Math.PI, Math.PI);
          } else {
            // Stop again for 5-10 seconds
            state.isStopped = true;
            state.timer = rng.range(5.0, 10.0);
            state.moveSpeed = 0;
          }
        }

        if (!state.isStopped && (state.moveSpeed || 0) > 0) {
          const spd = state.moveSpeed || 0.4;
          const hd = state.moveHeading || 0;
          obs.vx = -spd * Math.sin(hd);
          obs.vy = spd * Math.cos(hd);
          obs.heading = hd;
        } else {
          obs.vx = 0;
          obs.vy = 0;
        }
        break;
      }

      case 'debris': {
        // Static debris
        obs.vx = 0;
        obs.vy = 0;
        break;
      }

      default: {
        // Cars / trucks
        obs.vx = 0;
        obs.vy = state.baseVy || (obs.cls === 'truck' ? 4.5 : 7.0);
        obs.heading = 0;
        break;
      }
    }

    // Integrate position
    obs.x += obs.vx * dt;
    obs.y += obs.vy * dt;
  }
}
