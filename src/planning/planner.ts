import type { Vec2, World, VehicleState, Path, Obstacle } from '../core/types';

/**
 * OccupancyGrid structure exposed for debugging, HUD overlay, and 3D visualization.
 */
export interface OccupancyGrid {
  cols: number;
  rows: number;
  resolution: number; // 0.25 m
  originX: number;    // world x corresponding to col 0
  originY: number;    // world y corresponding to row 0
  data: Uint8Array;   // 0 = free drivable road, 1 = blocked (off-road or inflated obstacle)
  costMap: Float32Array; // cost penalty (clearance potential field) for heatmap rendering
  worldToGrid(x: number, y: number): { col: number; row: number } | null;
  gridToWorld(col: number, row: number): { x: number; y: number };
}

// Global debug export
export let lastGrid: OccupancyGrid | null = null;

// Vehicle & planning constants
const GRID_RES = 0.25;          // 0.25 m cells
const GRID_WIDTH = 20.0;        // 20 m wide
const GRID_AHEAD = 40.0;        // 40 m ahead
const GRID_BEHIND = 2.0;        // 2 m behind ego to cover vehicle footprint
const EGO_HALF_WIDTH = 0.9;     // typical car width 1.8 m / 2
const SAFETY_MARGIN = 0.5;      // 0.5 m margin
const INFLATION_DIST = EGO_HALF_WIDTH + SAFETY_MARGIN; // 1.4 m total inflation
const CLEARANCE_SAFE = 2.5;     // meters for road-center cost penalty
const PATH_SPACING = 0.5;       // ~0.5 m waypoint spacing
const TARGET_GOAL_DIST = 30.0;  // ~30 m ahead

/**
 * MinBinaryHeap for fast A* frontier traversal.
 */
class MinHeap {
  private data: { idx: number; f: number }[] = [];

  get size(): number {
    return this.data.length;
  }

  push(idx: number, f: number): void {
    this.data.push({ idx, f });
    this.bubbleUp(this.data.length - 1);
  }

  pop(): { idx: number; f: number } | undefined {
    if (this.data.length === 0) return undefined;
    const top = this.data[0];
    const bottom = this.data.pop()!;
    if (this.data.length > 0) {
      this.data[0] = bottom;
      this.sinkDown(0);
    }
    return top;
  }

  private bubbleUp(n: number): void {
    const element = this.data[n];
    while (n > 0) {
      const parentN = (n - 1) >> 1;
      const parent = this.data[parentN];
      if (element.f >= parent.f) break;
      this.data[n] = parent;
      this.data[parentN] = element;
      n = parentN;
    }
  }

  private sinkDown(n: number): void {
    const length = this.data.length;
    const element = this.data[n];
    while (true) {
      const leftChildN = (n << 1) + 1;
      const rightChildN = leftChildN + 1;
      let swapN = -1;
      let minF = element.f;

      if (leftChildN < length) {
        if (this.data[leftChildN].f < minF) {
          swapN = leftChildN;
          minF = this.data[leftChildN].f;
        }
      }

      if (rightChildN < length) {
        if (this.data[rightChildN].f < minF) {
          swapN = rightChildN;
        }
      }

      if (swapN === -1) break;
      this.data[n] = this.data[swapN];
      this.data[swapN] = element;
      n = swapN;
    }
  }
}

/**
 * Rasterizes road polygons into the occupancy grid using scanline intersection.
 * Road interior = 0 (free), everything else = 1 (blocked).
 */
function rasterizeRoadPolygons(grid: OccupancyGrid, roadPolygons: Vec2[][]): void {
  const { cols, rows, resolution, originX, originY, data } = grid;
  // Initialize to 1 (blocked / off-road)
  data.fill(1);

  // Scanline rasterization for each row
  for (let r = 0; r < rows; r++) {
    const yWorld = originY + (r + 0.5) * resolution;
    const xIntersections: number[] = [];

    for (const poly of roadPolygons) {
      const n = poly.length;
      for (let i = 0; i < n; i++) {
        const p1 = poly[i];
        const p2 = poly[(i + 1) % n];

        // Check if edge crosses the horizontal scanline y = yWorld
        if ((p1.y <= yWorld && p2.y > yWorld) || (p2.y <= yWorld && p1.y > yWorld)) {
          const t = (yWorld - p1.y) / (p2.y - p1.y);
          const x = p1.x + t * (p2.x - p1.x);
          xIntersections.push(x);
        }
      }
    }

    if (xIntersections.length < 2) continue;
    xIntersections.sort((a, b) => a - b);

    // Fill pairs of intersections as free
    for (let i = 0; i < xIntersections.length; i += 2) {
      const xStart = xIntersections[i];
      const xEnd = xIntersections[i + 1] ?? xStart;

      const cStart = Math.max(0, Math.floor((xStart - originX) / resolution));
      const cEnd = Math.min(cols - 1, Math.floor((xEnd - originX) / resolution));

      const rowOffset = r * cols;
      for (let c = cStart; c <= cEnd; c++) {
        data[rowOffset + c] = 0; // free
      }
    }
  }
}

/**
 * Rasterizes an oriented bounding box inflated by margin into the occupancy grid.
 */
function rasterizeOrientedBox(
  grid: OccupancyGrid,
  cx: number,
  cy: number,
  w: number,
  l: number,
  heading: number,
  margin: number
): void {
  const { cols, rows, resolution, originX, originY, data } = grid;

  const halfW = w / 2 + margin;
  const halfL = l / 2 + margin;
  const radius = Math.hypot(halfW, halfL);

  // Axis-aligned bounding box for quick rejection
  const minC = Math.max(0, Math.floor((cx - radius - originX) / resolution));
  const maxC = Math.min(cols - 1, Math.floor((cx + radius - originX) / resolution));
  const minR = Math.max(0, Math.floor((cy - radius - originY) / resolution));
  const maxR = Math.min(rows - 1, Math.floor((cy + radius - originY) / resolution));

  if (minC > maxC || minR > maxR) return;

  const cosH = Math.cos(heading);
  const sinH = Math.sin(heading);

  for (let r = minR; r <= maxR; r++) {
    const wy = originY + (r + 0.5) * resolution;
    const dy = wy - cy;
    const rowOffset = r * cols;

    for (let c = minC; c <= maxC; c++) {
      const wx = originX + (c + 0.5) * resolution;
      const dx = wx - cx;

      // Transform to local obstacle frame (heading 0 = +y, positive = CCW)
      const locY = -dx * sinH + dy * cosH;
      const locX = dx * cosH + dy * sinH;

      if (Math.abs(locX) <= halfW && Math.abs(locY) <= halfL) {
        data[rowOffset + c] = 1; // blocked
      }
    }
  }
}

/**
 * Inflates obstacles by ego half-width + 0.5 m, extending moving obstacles by 1.0 s.
 */
function inflateObstacles(grid: OccupancyGrid, obstacles: Obstacle[]): void {
  const { originY, rows, resolution } = grid;
  const gridMaxY = originY + rows * resolution;

  for (const obs of obstacles) {
    // Quick check if obstacle is far outside grid
    if (obs.y < originY - 15 || obs.y > gridMaxY + 15) continue;

    const speed = Math.hypot(obs.vx, obs.vy);

    if (speed < 0.2) {
      // Static obstacle
      rasterizeOrientedBox(grid, obs.x, obs.y, obs.w, obs.l, obs.heading, INFLATION_DIST);
    } else {
      // Moving obstacle: extend along velocity by 1.0 s (predictive footprint)
      // Sample poses at t = 0.0, 0.25, 0.5, 0.75, 1.0 s
      const numSteps = 5;
      for (let i = 0; i < numSteps; i++) {
        const t = (i / (numSteps - 1)) * 1.0;
        const px = obs.x + obs.vx * t;
        const py = obs.y + obs.vy * t;
        rasterizeOrientedBox(grid, px, py, obs.w, obs.l, obs.heading, INFLATION_DIST);
      }
    }
  }
}

/**
 * Computes Euclidean Chamfer Distance Transform and cost penalty.
 * Prefer road center of free space by assigning higher cost near blocked cells.
 */
function computeCostAndClearance(grid: OccupancyGrid): void {
  const { cols, rows, resolution, data, costMap } = grid;
  const total = cols * rows;

  // Distance field in grid units
  const dist = new Float32Array(total);
  const INF = 9999.0;

  for (let i = 0; i < total; i++) {
    dist[i] = data[i] === 1 ? 0 : INF;
  }

  // Pass 1: Forward raster (top-left to bottom-right)
  const SQRT2 = 1.41421356;
  for (let r = 0; r < rows; r++) {
    const rowOffset = r * cols;
    for (let c = 0; c < cols; c++) {
      const idx = rowOffset + c;
      if (dist[idx] === 0) continue;

      let d = dist[idx];
      if (c > 0) d = Math.min(d, dist[idx - 1] + 1);
      if (r > 0) {
        d = Math.min(d, dist[idx - cols] + 1);
        if (c > 0) d = Math.min(d, dist[idx - cols - 1] + SQRT2);
        if (c < cols - 1) d = Math.min(d, dist[idx - cols + 1] + SQRT2);
      }
      dist[idx] = d;
    }
  }

  // Pass 2: Backward raster (bottom-right to top-left)
  for (let r = rows - 1; r >= 0; r--) {
    const rowOffset = r * cols;
    for (let c = cols - 1; c >= 0; c--) {
      const idx = rowOffset + c;
      if (dist[idx] === 0) continue;

      let d = dist[idx];
      if (c < cols - 1) d = Math.min(d, dist[idx + 1] + 1);
      if (r < rows - 1) {
        d = Math.min(d, dist[idx + cols] + 1);
        if (c < cols - 1) d = Math.min(d, dist[idx + cols + 1] + SQRT2);
        if (c > 0) d = Math.min(d, dist[idx + cols - 1] + SQRT2);
      }
      dist[idx] = d;
    }
  }

  // Convert distance to meters and compute potential cost penalty
  for (let i = 0; i < total; i++) {
    if (data[i] === 1) {
      costMap[i] = 1000.0; // blocked
    } else {
      const distMeters = dist[i] * resolution;
      if (distMeters < CLEARANCE_SAFE) {
        const ratio = (CLEARANCE_SAFE - distMeters) / CLEARANCE_SAFE;
        // Quadratic penalty: highest near edges/obstacles, drops smoothly to 0 towards center
        costMap[i] = 15.0 * ratio * ratio;
      } else {
        costMap[i] = 0.0;
      }
    }
  }
}

/**
 * Finds the farthest reachable free goal cell about 30 m ahead.
 */
function findGoal(
  grid: OccupancyGrid,
  egoState: VehicleState
): { goalCol: number; goalRow: number; goalIdx: number } | null {
  const { cols, rows, resolution, originX, originY, data, costMap } = grid;

  const targetY = egoState.y + TARGET_GOAL_DIST;
  const targetRow = Math.min(rows - 2, Math.max(0, Math.round((targetY - originY) / resolution)));

  let bestIdx = -1;
  let bestCol = -1;
  let bestRow = -1;
  let bestScore = Infinity;

  // Search in a window around 30m ahead (rows 26m - 34m), or fallback down to 10m
  for (let searchOffset = 0; searchOffset <= 20; searchOffset++) {
    const row = targetRow - searchOffset;
    if (row < 0) break;

    const rowOffset = row * cols;
    for (let col = 0; col < cols; col++) {
      const idx = rowOffset + col;
      if (data[idx] === 0) {
        const wx = originX + (col + 0.5) * resolution;
        // Prefer low costMap (high clearance) and slight preference towards current vehicle alignment
        const score = costMap[idx] + 0.3 * Math.abs(wx - egoState.x);
        if (score < bestScore) {
          bestScore = score;
          bestIdx = idx;
          bestCol = col;
          bestRow = row;
        }
      }
    }

    // If we found candidates at the target distance, accept best
    if (bestIdx !== -1 && searchOffset <= 4) {
      break;
    }
  }

  if (bestIdx === -1) return null;
  return { goalCol: bestCol, goalRow: bestRow, goalIdx: bestIdx };
}

/**
 * 8-connected A* search finding optimal path through free space with clearance cost.
 */
function runAStar(
  grid: OccupancyGrid,
  startCol: number,
  startRow: number,
  goalCol: number,
  goalRow: number
): number[] | null {
  const { cols, rows, resolution, data, costMap } = grid;
  const total = cols * rows;

  const startIdx = startRow * cols + startCol;
  const goalIdx = goalRow * cols + goalCol;

  const gScore = new Float32Array(total).fill(1e9);
  const cameFrom = new Int32Array(total).fill(-1);
  const closedSet = new Uint8Array(total);

  const heap = new MinHeap();
  gScore[startIdx] = 0;

  // Heuristic: Octile distance
  const heuristic = (c: number, r: number) => {
    const dx = Math.abs(c - goalCol);
    const dy = Math.abs(r - goalRow);
    return (Math.max(dx, dy) + 0.4142 * Math.min(dx, dy)) * resolution;
  };

  heap.push(startIdx, heuristic(startCol, startRow));

  // 8-connected offsets
  const dCols = [-1, 0, 1, -1, 1, -1, 0, 1];
  const dRows = [-1, -1, -1, 0, 0, 1, 1, 1];
  const SQRT2 = 1.41421356;

  let farthestIdx = startIdx;
  let maxRowReached = startRow;

  while (heap.size > 0) {
    const current = heap.pop()!;
    const curIdx = current.idx;

    if (curIdx === goalIdx) {
      // Found exact goal! Reconstruct path
      return reconstructPath(curIdx, cameFrom);
    }

    if (closedSet[curIdx] === 1) continue;
    closedSet[curIdx] = 1;

    const curCol = curIdx % cols;
    const curRow = (curIdx / cols) | 0;

    if (curRow > maxRowReached) {
      maxRowReached = curRow;
      farthestIdx = curIdx;
    }

    const curG = gScore[curIdx];

    for (let k = 0; k < 8; k++) {
      const nc = curCol + dCols[k];
      const nr = curRow + dRows[k];

      if (nc < 0 || nc >= cols || nr < 0 || nr >= rows) continue;
      const nidx = nr * cols + nc;

      if (data[nidx] === 1 || closedSet[nidx] === 1) continue;

      // Prevent corner cutting through blocked diagonal corners
      const isDiag = dCols[k] !== 0 && dRows[k] !== 0;
      if (isDiag) {
        if (data[curRow * cols + nc] === 1 && data[nr * cols + curCol] === 1) {
          continue;
        }
      }

      const moveDist = isDiag ? resolution * SQRT2 : resolution;
      // Penalize moving backwards (driving in reverse)
      const backwardPenalty = dRows[k] < 0 ? 3.0 : 0.0;
      const stepCost = moveDist * (1.0 + costMap[nidx] + backwardPenalty);
      const tentativeG = curG + stepCost;

      if (tentativeG < gScore[nidx]) {
        gScore[nidx] = tentativeG;
        cameFrom[nidx] = curIdx;
        heap.push(nidx, tentativeG + heuristic(nc, nr));
      }
    }
  }

  // If goal not reachable, return path to farthest reachable node
  if (farthestIdx !== startIdx) {
    return reconstructPath(farthestIdx, cameFrom);
  }

  return null;
}

function reconstructPath(endIdx: number, cameFrom: Int32Array): number[] {
  const path: number[] = [];
  let curr = endIdx;
  while (curr !== -1) {
    path.push(curr);
    curr = cameFrom[curr];
  }
  path.reverse();
  return path;
}

/**
 * Line of sight check between two points using Bresenham algorithm on occupancy grid.
 */
function lineOfSight(grid: OccupancyGrid, c0: number, r0: number, c1: number, r1: number): boolean {
  const { cols, data } = grid;
  let dx = Math.abs(c1 - c0);
  let dy = Math.abs(r1 - r0);
  let sx = c0 < c1 ? 1 : -1;
  let sy = r0 < r1 ? 1 : -1;
  let err = dx - dy;

  let x = c0;
  let y = r0;

  while (true) {
    if (data[y * cols + x] === 1) return false;
    if (x === c1 && y === r1) break;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
  return true;
}

/**
 * Prunes redundant waypoints using line-of-sight shortcutting.
 */
function pruneWaypoints(grid: OccupancyGrid, rawIndices: number[]): Vec2[] {
  const { cols, resolution, originX, originY } = grid;
  if (rawIndices.length <= 2) {
    return rawIndices.map(idx => ({
      x: originX + (idx % cols + 0.5) * resolution,
      y: originY + (((idx / cols) | 0) + 0.5) * resolution,
    }));
  }

  const pruned: number[] = [rawIndices[0]];
  let currentIdx = 0;

  while (currentIdx < rawIndices.length - 1) {
    let furthest = currentIdx + 1;
    // Check line of sight ahead (lookahead up to 25 cells)
    const maxLookahead = Math.min(rawIndices.length - 1, currentIdx + 25);
    for (let test = maxLookahead; test > currentIdx + 1; test--) {
      const c0 = rawIndices[currentIdx] % cols;
      const r0 = (rawIndices[currentIdx] / cols) | 0;
      const c1 = rawIndices[test] % cols;
      const r1 = (rawIndices[test] / cols) | 0;

      if (lineOfSight(grid, c0, r0, c1, r1)) {
        furthest = test;
        break;
      }
    }

    pruned.push(rawIndices[furthest]);
    currentIdx = furthest;
  }

  return pruned.map(idx => ({
    x: originX + (idx % cols + 0.5) * resolution,
    y: originY + (((idx / cols) | 0) + 0.5) * resolution,
  }));
}

/**
 * Chaikin's corner cutting algorithm to produce a smooth quadratic B-spline curve.
 */
function chaikinSmooth(points: Vec2[], iterations: number = 2): Vec2[] {
  if (points.length < 3) return points;

  let current = points;
  for (let it = 0; it < iterations; it++) {
    const next: Vec2[] = [current[0]];
    for (let i = 0; i < current.length - 1; i++) {
      const p0 = current[i];
      const p1 = current[i + 1];

      // Q = 0.75 * p0 + 0.25 * p1
      const q: Vec2 = {
        x: 0.75 * p0.x + 0.25 * p1.x,
        y: 0.75 * p0.y + 0.25 * p1.y,
      };
      // R = 0.25 * p0 + 0.75 * p1
      const r: Vec2 = {
        x: 0.25 * p0.x + 0.75 * p1.x,
        y: 0.25 * p0.y + 0.75 * p1.y,
      };

      next.push(q, r);
    }
    next.push(current[current.length - 1]);
    current = next;
  }
  return current;
}

/**
 * Resamples a path at fixed spacing (~0.5 m).
 */
function resamplePath(points: Vec2[], spacing: number = PATH_SPACING): Path {
  if (points.length === 0) return [];
  if (points.length === 1) return [{ ...points[0] }];

  const resampled: Path = [{ ...points[0] }];
  let prev = points[0];
  let accumulatedDist = 0;

  for (let i = 1; i < points.length; i++) {
    const curr = points[i];
    const segDist = Math.hypot(curr.x - prev.x, curr.y - prev.y);
    if (segDist < 1e-4) continue;

    let d = spacing - accumulatedDist;
    while (d <= segDist) {
      const t = d / segDist;
      resampled.push({
        x: prev.x + t * (curr.x - prev.x),
        y: prev.y + t * (curr.y - prev.y),
      });
      d += spacing;
    }

    accumulatedDist = segDist - (d - spacing);
    prev = curr;
  }

  // Ensure end point is added if sufficiently distant
  const lastPoint = points[points.length - 1];
  const lastResampled = resampled[resampled.length - 1];
  if (Math.hypot(lastPoint.x - lastResampled.x, lastPoint.y - lastResampled.y) > spacing * 0.3) {
    resampled.push({ ...lastPoint });
  }

  return resampled;
}

/**
 * Verifies that all waypoints in the path are collision-free.
 */
function isPathCollisionFree(grid: OccupancyGrid, path: Path): boolean {
  const { cols, rows, resolution, originX, originY, data } = grid;
  for (const pt of path) {
    const c = Math.floor((pt.x - originX) / resolution);
    const r = Math.floor((pt.y - originY) / resolution);
    if (c < 0 || c >= cols || r < 0 || r >= rows) return false;
    if (data[r * cols + c] === 1) return false;
  }
  return true;
}

/**
 * Generates a short stopping path before the nearest blocker if no forward path exists.
 */
function generateStoppingPath(s: VehicleState, grid: OccupancyGrid): Path {
  const { cols, rows, resolution, originX, originY, data } = grid;

  // Scan directly ahead from ego position
  let maxClearY = s.y;
  for (let d = 0.5; d <= 5.0; d += 0.25) {
    const testY = s.y + d;
    const testX = s.x;
    const c = Math.floor((testX - originX) / resolution);
    const r = Math.floor((testY - originY) / resolution);

    if (c >= 0 && c < cols && r >= 0 && r < rows) {
      if (data[r * cols + c] === 1) {
        // Blocker hit; stop 1.0 m before it
        maxClearY = Math.max(s.y, testY - 1.0);
        break;
      }
      maxClearY = testY;
    }
  }

  const stopDist = maxClearY - s.y;
  const path: Path = [{ x: s.x, y: s.y }];
  if (stopDist > 0.4) {
    for (let y = s.y + PATH_SPACING; y <= maxClearY; y += PATH_SPACING) {
      path.push({ x: s.x, y });
    }
  }
  return path;
}

/**
 * Main planning function: plan(world, state) -> Path
 * 1. Rasterize unmarked road into 0.25m occupancy grid (40m ahead, 20m wide).
 * 2. Inflate obstacles by ego half-width + 0.5m margin, extend moving obstacles by 1.0s.
 * 3. 2-pass distance transform with cost penalty favoring road center.
 * 4. 8-connected A* search to farthest reachable free cell ~30m ahead.
 * 5. Smooth with Chaikin, resample at 0.5m, verify collision-free, fallback if failed.
 * 6. If blocked, return short stopping path.
 * Runs in under 15 ms.
 */
export function plan(w: World, s: VehicleState): Path {
  // Grid bounds: 20 m wide, 40 m ahead + 2 m behind ego
  const minX = s.x - GRID_WIDTH / 2;
  const maxX = s.x + GRID_WIDTH / 2;
  const minY = s.y - GRID_BEHIND;
  const maxY = s.y + GRID_AHEAD;

  const cols = Math.round((maxX - minX) / GRID_RES); // 80 cols
  const rows = Math.round((maxY - minY) / GRID_RES); // 168 rows
  const total = cols * rows;

  const data = new Uint8Array(total);
  const costMap = new Float32Array(total);

  const grid: OccupancyGrid = {
    cols,
    rows,
    resolution: GRID_RES,
    originX: minX,
    originY: minY,
    data,
    costMap,
    worldToGrid(x: number, y: number) {
      const col = Math.floor((x - minX) / GRID_RES);
      const row = Math.floor((y - minY) / GRID_RES);
      if (col < 0 || col >= cols || row < 0 || row >= rows) return null;
      return { col, row };
    },
    gridToWorld(col: number, row: number) {
      return {
        x: minX + (col + 0.5) * GRID_RES,
        y: minY + (row + 0.5) * GRID_RES,
      };
    },
  };

  // Expose debug grid for Member D heatmap visualization
  lastGrid = grid;
  (plan as any).lastGrid = grid;

  // 1. Rasterize road drivable area
  rasterizeRoadPolygons(grid, w.roadPolygons);

  // 2. Inflate obstacles and extend moving obstacle predictive footprints
  inflateObstacles(grid, w.obstacles);

  // 3. Compute distance field and clearance cost penalty
  computeCostAndClearance(grid);

  // 4. Locate start cell and goal cell
  let startCol = Math.round((s.x - minX) / GRID_RES);
  let startRow = Math.round((s.y - minY) / GRID_RES);
  startCol = Math.max(0, Math.min(cols - 1, startCol));
  startRow = Math.max(0, Math.min(rows - 1, startRow));

  // If ego start cell is temporarily marked blocked, find nearest free cell
  if (grid.data[startRow * cols + startCol] === 1) {
    let foundFree = false;
    for (let r = 1; r <= 3; r++) {
      for (let dc = -r; dc <= r && !foundFree; dc++) {
        for (let dr = -r; dr <= r && !foundFree; dr++) {
          const tc = startCol + dc;
          const tr = startRow + dr;
          if (tc >= 0 && tc < cols && tr >= 0 && tr < rows) {
            if (grid.data[tr * cols + tc] === 0) {
              startCol = tc;
              startRow = tr;
              foundFree = true;
            }
          }
        }
      }
    }
  }

  // Find best goal ~30 m ahead
  const goal = findGoal(grid, s);
  if (!goal) {
    // Road completely blocked ahead: return safe stopping path
    return generateStoppingPath(s, grid);
  }

  // 5. Run 8-connected A* search
  const rawCellIndices = runAStar(grid, startCol, startRow, goal.goalCol, goal.goalRow);
  if (!rawCellIndices || rawCellIndices.length < 2) {
    // No path found: return safe stopping path
    return generateStoppingPath(s, grid);
  }

  // Convert raw indices to world coordinates
  const rawWaypoints: Vec2[] = rawCellIndices.map(idx => ({
    x: minX + ((idx % cols) + 0.5) * GRID_RES,
    y: minY + (((idx / cols) | 0) + 0.5) * GRID_RES,
  }));

  // Force first point to exact ego position
  rawWaypoints[0] = { x: s.x, y: s.y };

  // 6. Shortcut / prune waypoints
  const pruned = pruneWaypoints(grid, rawCellIndices);
  pruned[0] = { x: s.x, y: s.y };

  // 7. Smooth with Chaikin's algorithm
  const smoothed = chaikinSmooth(pruned, 2);
  smoothed[0] = { x: s.x, y: s.y };

  // 8. Resample at fixed 0.5 m spacing
  const resampledSmoothed = resamplePath(smoothed, PATH_SPACING);

  // 9. Collision verification: verify all resampled points are collision-free
  if (isPathCollisionFree(grid, resampledSmoothed)) {
    return resampledSmoothed;
  }

  // Fallback: resample the raw unsmoothed A* path
  const resampledRaw = resamplePath(rawWaypoints, PATH_SPACING);
  if (isPathCollisionFree(grid, resampledRaw)) {
    return resampledRaw;
  }

  // If raw also failed, return safe stopping path
  return generateStoppingPath(s, grid);
}

/**
 * Planner namespace object for Member D import compatibility.
 */
export const planner = {
  plan,
  get lastGrid() {
    return lastGrid;
  },
};
