import { Obstacle, Vec2, VehicleState } from '../core/types';

export const EGO_WIDTH = 1.8; // meters
export const EGO_LENGTH = 4.2; // meters

/**
 * Returns the 4 corner points of an oriented bounding box in 2D world coordinates.
 * Convention:
 * - x = right, y = forward, theta = 0 facing +y, CCW positive.
 * - forward unit vector: (-sin theta, cos theta)
 * - right unit vector:   (cos theta, sin theta)
 */
export function getOrientedBoxCorners(
  centerX: number,
  centerY: number,
  width: number,
  length: number,
  heading: number
): [Vec2, Vec2, Vec2, Vec2] {
  const sinT = Math.sin(heading);
  const cosT = Math.cos(heading);

  // Unit vectors
  const fX = -sinT;
  const fY = cosT;
  const rX = cosT;
  const rY = sinT;

  const halfL = length / 2;
  const halfW = width / 2;

  // 4 corners: front-left, front-right, rear-right, rear-left
  return [
    { x: centerX + halfL * fX - halfW * rX, y: centerY + halfL * fY - halfW * rY },
    { x: centerX + halfL * fX + halfW * rX, y: centerY + halfL * fY + halfW * rY },
    { x: centerX - halfL * fX + halfW * rX, y: centerY - halfL * fY + halfW * rY },
    { x: centerX - halfL * fX - halfW * rX, y: centerY - halfL * fY - halfW * rY },
  ];
}

/**
 * Separating Axis Theorem (SAT) test for two oriented bounding boxes.
 * Returns true if the two boxes overlap.
 */
export function checkOrientedBoxOverlap(
  boxA: [Vec2, Vec2, Vec2, Vec2],
  boxB: [Vec2, Vec2, Vec2, Vec2]
): boolean {
  // Test axes from Box A and Box B edges
  const axes: Vec2[] = [];

  // Edges of Box A
  for (let i = 0; i < 2; i++) {
    const p1 = boxA[i];
    const p2 = boxA[(i + 1) % 4];
    const edge = { x: p2.x - p1.x, y: p2.y - p1.y };
    // Normal to edge (-edge.y, edge.x)
    const len = Math.hypot(edge.x, edge.y);
    if (len > 1e-6) {
      axes.push({ x: -edge.y / len, y: edge.x / len });
    }
  }

  // Edges of Box B
  for (let i = 0; i < 2; i++) {
    const p1 = boxB[i];
    const p2 = boxB[(i + 1) % 4];
    const edge = { x: p2.x - p1.x, y: p2.y - p1.y };
    const len = Math.hypot(edge.x, edge.y);
    if (len > 1e-6) {
      axes.push({ x: -edge.y / len, y: edge.x / len });
    }
  }

  // Check overlap on each axis
  for (const axis of axes) {
    let minA = Infinity;
    let maxA = -Infinity;
    for (const p of boxA) {
      const proj = p.x * axis.x + p.y * axis.y;
      if (proj < minA) minA = proj;
      if (proj > maxA) maxA = proj;
    }

    let minB = Infinity;
    let maxB = -Infinity;
    for (const p of boxB) {
      const proj = p.x * axis.x + p.y * axis.y;
      if (proj < minB) minB = proj;
      if (proj > maxB) maxB = proj;
    }

    // If there is a gap along this axis, no overlap
    if (maxA < minB || maxB < minA) {
      return false;
    }
  }

  return true;
}

/**
 * Checks collision between the ego vehicle and a single obstacle.
 */
export function isVehicleCollidingWithObstacle(
  ego: VehicleState,
  obs: Obstacle,
  egoW: number = EGO_WIDTH,
  egoL: number = EGO_LENGTH
): boolean {
  const egoCorners = getOrientedBoxCorners(ego.x, ego.y, egoW, egoL, ego.theta);
  const obsCorners = getOrientedBoxCorners(obs.x, obs.y, obs.w, obs.l, obs.heading);
  return checkOrientedBoxOverlap(egoCorners, obsCorners);
}

/**
 * Result of collision detection across all obstacles in the world
 */
export interface CollisionResult {
  isColliding: boolean;
  collidingObstacleIds: string[];
  minObstacleDistance: number;
}

/**
 * Checks collisions between the ego vehicle and all obstacles in the world,
 * returning collision status, IDs, and minimum distance to any obstacle.
 */
export function evaluateCollisions(
  ego: VehicleState,
  obstacles: Obstacle[],
  egoW: number = EGO_WIDTH,
  egoL: number = EGO_LENGTH
): CollisionResult {
  let isColliding = false;
  const collidingObstacleIds: string[] = [];
  let minObstacleDistance = Infinity;

  const egoCorners = getOrientedBoxCorners(ego.x, ego.y, egoW, egoL, ego.theta);

  for (const obs of obstacles) {
    // Distance between centers
    const centerDist = Math.hypot(obs.x - ego.x, obs.y - ego.y);
    const approxDist = Math.max(0, centerDist - (egoL / 2 + obs.l / 2));
    if (approxDist < minObstacleDistance) {
      minObstacleDistance = approxDist;
    }

    const obsCorners = getOrientedBoxCorners(obs.x, obs.y, obs.w, obs.l, obs.heading);
    if (checkOrientedBoxOverlap(egoCorners, obsCorners)) {
      isColliding = true;
      collidingObstacleIds.push(obs.id);
    }
  }

  return {
    isColliding,
    collidingObstacleIds,
    minObstacleDistance: minObstacleDistance === Infinity ? 999.0 : minObstacleDistance,
  };
}
