// src/perception/projection.ts
// Flat-ground homography projection converting 2D image coordinates to top-down world coordinates (meters).
// SIH 2026 - Autonomous Car Simulation on Unmarked Indian Roads (Member C)

import type { Vec2, Obstacle, ObstacleClass } from '../core/types';

export interface CameraConfig {
  height: number;        // Camera height above ground in meters (default: 1.4 m)
  horizonRatio: number;  // Horizon vertical position ratio of image height (default: 0.32 * H)
  fx: number;            // Focal length X in pixels for 1920x1080 (default: 1000 px)
  fy: number;            // Focal length Y in pixels (default: 1000 px)
  cx: number;            // Principal point X in pixels (default: W / 2 = 960)
  maxDistance: number;   // Max forward distance clip in meters (default: 40.0 m)
  minDistance: number;   // Min forward distance clip in meters (default: 0.5 m)
  imageWidth: number;    // Frame width (default: 1920)
  imageHeight: number;   // Frame height (default: 1080)
}

export const DEFAULT_CAMERA_CONFIG: CameraConfig = {
  height: 1.4,
  horizonRatio: 0.32,
  fx: 1000.0,
  fy: 1000.0,
  cx: 960.0,
  maxDistance: 40.0,
  minDistance: 0.5,
  imageWidth: 1920,
  imageHeight: 1080
};

// Default physical dimensions [width, length] in meters per ObstacleClass
export const DEFAULT_OBSTACLE_DIMENSIONS: Record<ObstacleClass, { w: number; l: number }> = {
  pedestrian: { w: 0.6, l: 0.6 },
  two_wheeler: { w: 0.8, l: 1.8 },
  auto: { w: 1.4, l: 2.6 },
  car: { w: 1.8, l: 4.2 },
  truck: { w: 2.5, l: 7.5 },
  cow: { w: 1.1, l: 2.1 },
  debris: { w: 0.8, l: 0.8 }
};

export type Point2D = [number, number] | { x: number; y: number };

export interface RawObstacleInput {
  cls: ObstacleClass | string;
  bbox: [number, number, number, number]; // [x1, y1, x2, y2]
  id?: string;
  confidence?: number;
}

export interface RawFrameData {
  frame?: number;
  timestamp?: number;
  image?: string;
  road: Point2D[] | Point2D[][];
  obstacles: RawObstacleInput[];
}

/**
 * Normalizes input point into [u, v] pixel coordinates.
 */
function normalizePoint(pt: Point2D): [number, number] {
  if (Array.isArray(pt)) {
    return [pt[0], pt[1]];
  }
  return [pt.x, pt.y];
}

/**
 * Maps arbitrary or noisy class labels to the official ObstacleClass union.
 */
export function normalizeObstacleClass(clsStr: string): ObstacleClass {
  const norm = clsStr.toLowerCase().replace(/[-_]/g, '');
  if (norm.includes('pedestrian') || norm.includes('person') || norm.includes('human')) return 'pedestrian';
  if (norm.includes('two') || norm.includes('wheel') || norm.includes('bike') || norm.includes('cycle') || norm.includes('scooter')) return 'two_wheeler';
  if (norm.includes('auto') || norm.includes('rickshaw') || norm.includes('tuktuk')) return 'auto';
  if (norm.includes('truck') || norm.includes('bus') || norm.includes('lorry')) return 'truck';
  if (norm.includes('cow') || norm.includes('cattle') || norm.includes('animal') || norm.includes('dog')) return 'cow';
  if (norm.includes('debris') || norm.includes('obstacle') || norm.includes('barrier') || norm.includes('rock')) return 'debris';
  return 'car'; // default fallback for 4-wheel vehicles
}

/**
 * Converts image pixel (u, v) on the road plane to top-down world coordinates (x, y) in meters.
 * World frame:
 *   x = right (meters)
 *   y = forward (meters, 0 at ego bumper, up to 40m)
 * Returns null if the pixel is above the horizon or exceeds maxDistance.
 */
export function projectPixelToWorld(
  u: number,
  v: number,
  customConfig?: Partial<CameraConfig>
): Vec2 | null {
  const cfg = { ...DEFAULT_CAMERA_CONFIG, ...customConfig };
  const vHorizon = cfg.horizonRatio * cfg.imageHeight;

  const deltaV = v - vHorizon;
  if (deltaV <= 0.1) {
    // Point is at or above horizon (sky / infinite distance)
    return null;
  }

  // Flat ground geometry: tan(pitch) = deltaV / fy = height / yWorld
  const yWorld = (cfg.height * cfg.fy) / deltaV;

  if (yWorld > cfg.maxDistance || yWorld < cfg.minDistance) {
    return null;
  }

  // Lateral geometry: xWorld / yWorld = (u - cx) / fx
  const xWorld = yWorld * ((u - cfg.cx) / cfg.fx);

  return { x: xWorld, y: yWorld };
}

/**
 * Inverse projection: converts top-down world meters (x, y) to image pixel (u, v).
 * Useful for drawing overlays on top of the raw camera frame.
 */
export function projectWorldToPixel(
  x: number,
  y: number,
  customConfig?: Partial<CameraConfig>
): { u: number; v: number } | null {
  if (y <= 0.1) return null;
  const cfg = { ...DEFAULT_CAMERA_CONFIG, ...customConfig };
  const vHorizon = cfg.horizonRatio * cfg.imageHeight;

  const deltaV = (cfg.height * cfg.fy) / y;
  const v = vHorizon + deltaV;
  const u = cfg.cx + (x * cfg.fx) / y;

  return { u, v };
}

/**
 * Clips a polygon against the minimum pixel row corresponding to maxDistance (yWorld <= 40 m).
 * Uses Sutherland-Hodgman clipping algorithm.
 */
function clipPolygonAboveHorizon(
  points: [number, number][],
  minV: number,
  maxV: number
): [number, number][] {
  if (points.length < 3) return points;

  // Clip against bottom horizon limit (v >= minV)
  let outputList = points;

  // Clip against v >= minV
  let inputList = outputList;
  outputList = [];
  if (inputList.length === 0) return [];

  let s = inputList[inputList.length - 1];
  for (const e of inputList) {
    const eInside = e[1] >= minV;
    const sInside = s[1] >= minV;

    if (eInside) {
      if (!sInside) {
        // Line intersects minV: interpolate u
        const t = (minV - s[1]) / (e[1] - s[1]);
        const intersectU = s[0] + t * (e[0] - s[0]);
        outputList.push([intersectU, minV]);
      }
      outputList.push(e);
    } else if (sInside) {
      const t = (minV - s[1]) / (e[1] - s[1]);
      const intersectU = s[0] + t * (e[0] - s[0]);
      outputList.push([intersectU, minV]);
    }
    s = e;
  }

  return outputList;
}

/**
 * Converts a road polygon in image pixels to top-down world coordinates (meters).
 * Clips at maxDistance (40 m).
 */
export function projectRoadPolygon(
  rawPoints: Point2D[],
  customConfig?: Partial<CameraConfig>
): Vec2[] {
  const cfg = { ...DEFAULT_CAMERA_CONFIG, ...customConfig };
  const vHorizon = cfg.horizonRatio * cfg.imageHeight;

  // v value corresponding to yWorld = maxDistance
  const minV = vHorizon + (cfg.height * cfg.fy) / cfg.maxDistance;
  const maxV = cfg.imageHeight;

  const normalized = rawPoints.map(normalizePoint);
  const clipped = clipPolygonAboveHorizon(normalized, minV, maxV);

  const worldPoints: Vec2[] = [];
  for (const [u, v] of clipped) {
    const pt = projectPixelToWorld(u, v, cfg);
    if (pt) {
      worldPoints.push(pt);
    }
  }

  // Extend road base to enclose ego origin (y = 0) since front camera field of view
  // starts at the bumper (y ≈ 1.9m). This provides a continuous drivable corridor for the planner.
  if (worldPoints.length >= 3) {
    const nearPoints = worldPoints.filter(p => p.y < 3.5);
    if (nearPoints.length >= 2) {
      const minX = Math.min(...nearPoints.map(p => p.x));
      const maxX = Math.max(...nearPoints.map(p => p.x));
      worldPoints.push({ x: maxX, y: -1.0 });
      worldPoints.push({ x: minX, y: -1.0 });
    }
  }

  return worldPoints;
}

/**
 * Converts raw bounding box [x1, y1, x2, y2] to top-down Obstacle.
 * Ground contact point is the bottom-centre: ((x1 + x2)/2, y2).
 */
export function projectObstacle(
  raw: RawObstacleInput,
  id: string,
  customConfig?: Partial<CameraConfig>
): Obstacle | null {
  const cfg = { ...DEFAULT_CAMERA_CONFIG, ...customConfig };
  const [x1, y1, x2, y2] = raw.bbox;

  const uBottomCenter = (x1 + x2) / 2;
  const vBottomCenter = y2;

  const pos = projectPixelToWorld(uBottomCenter, vBottomCenter, cfg);
  if (!pos) {
    return null; // clipped or out of bounds
  }

  const cls = normalizeObstacleClass(String(raw.cls));
  const dims = DEFAULT_OBSTACLE_DIMENSIONS[cls] || { w: 1.5, l: 3.5 };

  return {
    id,
    cls,
    x: pos.x,
    y: pos.y,
    w: dims.w,
    l: dims.l,
    heading: 0, // facing +y (forward) by default
    vx: 0,
    vy: 0
  };
}

/**
 * Converts full raw frame data to a complete World state for the simulator.
 */
export function projectRawFrameToWorld(
  frameData: RawFrameData,
  customConfig?: Partial<CameraConfig>,
  t = 0
): { roadPolygons: Vec2[][]; obstacles: Obstacle[]; t: number } {
  const cfg = { ...DEFAULT_CAMERA_CONFIG, ...customConfig };

  // 1. Process road polygons
  const roadPolygons: Vec2[][] = [];
  if (Array.isArray(frameData.road) && frameData.road.length > 0) {
    // Check if road is Point2D[][] or Point2D[]
    const firstItem = frameData.road[0];
    if (Array.isArray(firstItem) && (Array.isArray(firstItem[0]) || typeof firstItem[0] === 'object')) {
      for (const poly of frameData.road as Point2D[][]) {
        const proj = projectRoadPolygon(poly, cfg);
        if (proj.length >= 3) {
          roadPolygons.push(proj);
        }
      }
    } else {
      const proj = projectRoadPolygon(frameData.road as Point2D[], cfg);
      if (proj.length >= 3) {
        roadPolygons.push(proj);
      }
    }
  }

  // 2. Process obstacles
  const obstacles: Obstacle[] = [];
  if (Array.isArray(frameData.obstacles)) {
    frameData.obstacles.forEach((rawObs, idx) => {
      const obsId = rawObs.id || `obs_${frameData.frame ?? 0}_${idx + 1}`;
      const obs = projectObstacle(rawObs, obsId, cfg);
      if (obs) {
        obstacles.push(obs);
      }
    });
  }

  return {
    roadPolygons,
    obstacles,
    t: frameData.timestamp ?? t
  };
}
