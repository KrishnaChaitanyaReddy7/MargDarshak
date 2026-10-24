<<<<<<< HEAD
import { World } from '../core/types';
import { createWorld, stepWorld } from '../planning/world';

/**
 * Loads recorded sensor replay frames from a URL or generates
 * pre-recorded synthetic playback worlds.
 *
 * Exact Contract Signature:
 * loadReplay(url: string): Promise<World[]>
 *
 * @param url Resource URL or scenario descriptor for playback
 * @returns Array of World states representing time snapshots
 */
export async function loadReplay(url: string): Promise<World[]> {
  // If fetching from a real JSON endpoint:
  if (url.startsWith('http://') || url.startsWith('https://') || url.endsWith('.json')) {
    try {
      const response = await fetch(url);
      if (response.ok) {
        const data = await response.json();
        if (Array.isArray(data)) {
          return data as World[];
        }
      }
    } catch {
      // Fallback to generated synthetic frames if fetch fails
    }
  }

  // Generate synthetic replay sequence for offline demonstration
  const frames: World[] = [];
  const baseWorld = createWorld('dense_mixed');
  const dt = 1 / 60;
  const numFrames = 300; // 5 seconds of replay

  let currentWorld = JSON.parse(JSON.stringify(baseWorld)) as World;

  for (let f = 0; f < numFrames; f++) {
    stepWorld(currentWorld, dt);
    frames.push(JSON.parse(JSON.stringify(currentWorld)));
  }

  return frames;
=======
// src/perception/replay.ts
// Replay loader and velocity estimator for SIH 2026 Autonomous Car Simulation (Member C)

import type { World, Obstacle, VehicleState } from '../core/types';
import {
  projectRawFrameToWorld,
  type RawFrameData,
  type CameraConfig,
  DEFAULT_CAMERA_CONFIG
} from './projection';

export interface ReplayIndex {
  fps?: number;
  width?: number;
  height?: number;
  camera?: Partial<CameraConfig>;
  totalFrames?: number;
  frames: string[];
}

// In-memory cache for loaded replay worlds
let replayWorldsCache: World[] = [];
let replayRawFramesCache: RawFrameData[] = [];
let replayCameraConfig: CameraConfig = { ...DEFAULT_CAMERA_CONFIG };

/**
 * Loads a replay dataset from a given URL (index.json or replay directory).
 * Projects all road polygons and obstacles, and estimates obstacle velocities
 * by differencing consecutive frames with nearest-neighbour matching.
 *
 * @param url URL to index.json or folder (e.g. "/replay/index.json")
 * @returns Promise<World[]> Array of World states, one per frame
 */
export async function loadReplay(
  url: string,
  customCamera?: Partial<CameraConfig>
): Promise<World[]> {
  let indexUrl = url;
  if (!indexUrl.endsWith('.json')) {
    indexUrl = indexUrl.replace(/\/+$/, '') + '/index.json';
  }

  // Derive base path from indexUrl (e.g. "/replay/")
  const lastSlash = indexUrl.lastIndexOf('/');
  const basePath = lastSlash >= 0 ? indexUrl.substring(0, lastSlash + 1) : '';

  const res = await fetch(indexUrl);
  if (!res.ok) {
    throw new Error(`Failed to load replay index from ${indexUrl}: ${res.statusText}`);
  }

  const indexData = (await res.json()) as ReplayIndex | RawFrameData;

  // Check if user passed a single frame JSON directly
  if (!('frames' in indexData) && 'road' in indexData) {
    const singleFrame = indexData as RawFrameData;
    const world = projectRawFrameToWorld(singleFrame, customCamera, 0);
    replayWorldsCache = [world];
    replayRawFramesCache = [singleFrame];
    return replayWorldsCache;
  }

  const index = indexData as ReplayIndex;
  const frameFiles = index.frames || [];
  const fps = index.fps && index.fps > 0 ? index.fps : 15;
  const dtNominal = 1 / fps;

  if (index.camera) {
    replayCameraConfig = { ...DEFAULT_CAMERA_CONFIG, ...index.camera, ...customCamera };
  } else if (customCamera) {
    replayCameraConfig = { ...DEFAULT_CAMERA_CONFIG, ...customCamera };
  }

  // Fetch all frame JSON files concurrently
  const framePromises = frameFiles.map(async (fName, i) => {
    const fUrl = fName.startsWith('http') || fName.startsWith('/')
      ? fName
      : `${basePath}${fName}`;
    const frameRes = await fetch(fUrl);
    if (!frameRes.ok) {
      throw new Error(`Failed to fetch replay frame ${fUrl}: ${frameRes.statusText}`);
    }
    const frameJson = (await frameRes.json()) as RawFrameData;
    if (frameJson.timestamp === undefined) {
      frameJson.timestamp = i * dtNominal;
    }
    return frameJson;
  });

  const rawFrames = await Promise.all(framePromises);
  replayRawFramesCache = rawFrames;

  // Project raw frames into initial World states
  const rawWorlds: World[] = rawFrames.map((raw, idx) => {
    const t = raw.timestamp ?? idx * dtNominal;
    return projectRawFrameToWorld(raw, replayCameraConfig, t);
  });

  // Perform multi-frame tracking and velocity estimation
  const trackedWorlds = estimateVelocitiesAndTrack(rawWorlds, dtNominal);

  replayWorldsCache = trackedWorlds;
  return replayWorldsCache;
}

/**
 * Estimates obstacles' vx, vy and heading across consecutive frames
 * using nearest-neighbour Euclidean association and maintains stable IDs.
 */
export function estimateVelocitiesAndTrack(
  worlds: World[],
  nominalDt = 1 / 15,
  maxMatchDistance = 5.0 // maximum displacement between frames for same vehicle (5m at 15fps is 75m/s)
): World[] {
  if (worlds.length === 0) return [];

  let nextTrackId = 1;
  const result: World[] = [];

  // Frame 0: assign initial IDs and zero velocity
  const frame0Obstacles = worlds[0].obstacles.map((obs) => ({
    ...obs,
    id: `track_${nextTrackId++}`,
    vx: 0,
    vy: 0,
    heading: 0
  }));

  result.push({
    ...worlds[0],
    obstacles: frame0Obstacles
  });

  // Track across consecutive frames
  for (let k = 1; k < worlds.length; k++) {
    const prevWorld = result[k - 1];
    const currWorld = worlds[k];

    let dt = currWorld.t - prevWorld.t;
    if (dt <= 0.001 || !Number.isFinite(dt)) {
      dt = nominalDt;
    }

    const prevObstacles = prevWorld.obstacles;
    const currObstacles = currWorld.obstacles;

    const matchedPrevIndices = new Set<number>();
    const trackedCurrent: Obstacle[] = [];

    for (const curr of currObstacles) {
      let bestDist = Infinity;
      let bestPrevIdx = -1;

      // Find nearest previous obstacle with matching class
      for (let p = 0; p < prevObstacles.length; p++) {
        if (matchedPrevIndices.has(p)) continue;
        const prev = prevObstacles[p];

        // Match if classes are identical or compatible
        if (prev.cls !== curr.cls) continue;

        const dx = curr.x - prev.x;
        const dy = curr.y - prev.y;
        const dist = Math.hypot(dx, dy);

        if (dist < bestDist && dist <= maxMatchDistance) {
          bestDist = dist;
          bestPrevIdx = p;
        }
      }

      if (bestPrevIdx >= 0) {
        // Match found: reuse stable track ID and estimate velocity
        matchedPrevIndices.add(bestPrevIdx);
        const prev = prevObstacles[bestPrevIdx];

        const vx = (curr.x - prev.x) / dt;
        const vy = (curr.y - prev.y) / dt;
        const speed = Math.hypot(vx, vy);

        // Heading: 0 means facing +y (forward), positive = counter-clockwise (left)
        // theta = atan2(-vx, vy)
        let heading = 0;
        if (speed > 0.4) {
          heading = Math.atan2(-vx, vy);
        } else {
          heading = prev.heading; // retain previous orientation if stationary
        }

        trackedCurrent.push({
          ...curr,
          id: prev.id,
          vx: Math.round(vx * 100) / 100,
          vy: Math.round(vy * 100) / 100,
          heading: Math.round(heading * 1000) / 1000
        });
      } else {
        // New obstacle entered the scene: assign new track ID
        trackedCurrent.push({
          ...curr,
          id: `track_${nextTrackId++}`,
          vx: 0,
          vy: 0,
          heading: 0
        });
      }
    }

    result.push({
      ...currWorld,
      obstacles: trackedCurrent
    });
  }

  return result;
}

/**
 * Replay mode hook: agreed single function interface with Member A.
 * Returns the World state for replay frame i.
 * Clamps i to valid bounds [0, totalFrames - 1].
 */
export function getReplayWorld(i: number): World {
  if (replayWorldsCache.length === 0) {
    // Return empty fallback world if replay is not yet loaded
    return {
      roadPolygons: [],
      obstacles: [],
      t: 0
    };
  }

  const safeIdx = Math.max(0, Math.min(Math.floor(i), replayWorldsCache.length - 1));
  const world = replayWorldsCache[safeIdx];

  // Deep clone to ensure consumer mutations don't alter cached replay data
  return {
    roadPolygons: world.roadPolygons.map((poly) => poly.map((pt) => ({ ...pt }))),
    obstacles: world.obstacles.map((obs) => ({ ...obs })),
    t: world.t
  };
}

/**
 * Resets the ego vehicle state to the origin (0, 0, theta=0) for replay mode.
 * In replay mode, every frame is an egocentric observation from the vehicle's
 * current frame of reference.
 */
export function resetEgoForReplay(): VehicleState {
  return {
    x: 0,
    y: 0,
    theta: 0, // facing forward (+y)
    v: 0,
    delta: 0
  };
}

/**
 * Sets preloaded worlds into the replay cache (useful for testing or synthetic replays).
 */
export function setReplayWorlds(worlds: World[]): void {
  replayWorldsCache = worlds;
}

/**
 * Returns total count of frames loaded in replay mode.
 */
export function getReplayFrameCount(): number {
  return replayWorldsCache.length;
}

/**
 * Checks whether replay data is loaded and ready.
 */
export function isReplayLoaded(): boolean {
  return replayWorldsCache.length > 0;
}

/**
 * Returns raw frame data for side-panel camera inspection.
 */
export function getRawReplayFrame(i: number): RawFrameData | null {
  if (replayRawFramesCache.length === 0) return null;
  const safeIdx = Math.max(0, Math.min(Math.floor(i), replayRawFramesCache.length - 1));
  return replayRawFramesCache[safeIdx];
}

/**
 * Returns current camera configuration used in replay projection.
 */
export function getReplayCameraConfig(): CameraConfig {
  return { ...replayCameraConfig };
>>>>>>> f2a7871d1e12e3fcc0ada4eb7465a19d47dc69cd
}
