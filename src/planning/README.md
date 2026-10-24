# Planning Module (/src/planning) - SIH 2026

**Role:** Member B  
**Component:** World Simulation & Real-Time Path Planner for Unmarked Indian Roads  

---

## 1. Overview

This module provides the simulation world and dynamic local path planner for an autonomous vehicle navigating unmarked Indian roads (no lane lines, ragged dirt shoulders, dense mixed traffic).

### Module Architecture

- [world.ts](file:///c:/Users/Venkat/Documents/HMTV/src/planning/world.ts):
  - `createWorld(scenario: string): World`: Initializes 3 procedural scenarios:
    - `"straight"`: 8 m wide road with ragged dirt shoulders (noisy edges).
    - `"curve"`: Gentle S-bend with varying road width (6–10 m).
    - `"chaos"`: Wide road (11–12 m) with dense mixed Indian traffic.
  - `stepWorld(w: World, dt: number): void`: Updates dynamic obstacle trajectories using deterministic seeded RNG (Mulberry32).
  - `addObstacle(w: World, cls: ObstacleClass, x: number, y: number): Obstacle`: Interactive obstacle spawning (e.g., UI clicks).
  - **Obstacle Behaviors**:
    - **Auto-rickshaws** ($1.4 \times 2.6\text{ m}$): Lateral sinusoidal weaving inside road boundaries.
    - **Two-wheelers** ($0.7 \times 1.9\text{ m}$): Fast forward speed with sharp zigzag maneuvers.
    - **Pedestrians** ($0.5 \times 0.5\text{ m}$): Lateral road crossing with pauses.
    - **Cows** ($0.8 \times 2.0\text{ m}$): Stop in the middle of the road for 5–10 s, then move slowly.
    - **Parked Debris** ($1.0 \times 1.0\text{ m}$): Static road obstacles.

- [planner.ts](file:///c:/Users/Venkat/Documents/HMTV/src/planning/planner.ts):
  - `plan(w: World, s: VehicleState): Path`:
    1. **Occupancy Grid**: Rasterizes drivable road polygon into a $0.25\text{ m}$ resolution grid spanning $20\text{ m}$ laterally and $40\text{ m}$ ahead of the vehicle ($80 \times 168 = 13,440$ cells).
    2. **Obstacle Inflation & Predictive Footprints**: Inflates obstacles by ego half-width ($0.9\text{ m}$) + safety margin ($0.5\text{ m}$) = $1.4\text{ m}$. Moving obstacles are extended along their velocity vector by $1.0\text{ s}$ to prevent cutting off moving traffic.
    3. **Distance Transform & Road Center Cost**: 2-pass Chamfer distance transform produces a clearance potential field. Cells near edges and obstacles incur a quadratic cost penalty, directing the vehicle naturally down the open center of free space.
    4. **8-Connected A\***: Searches for an optimal path to the farthest reachable free cell ~30 m ahead. Backward motion is penalized.
    5. **Smoothing & Resampling**: Line-of-sight shortcutting, Chaikin corner-cutting (quadratic B-spline), and uniform $0.5\text{ m}$ waypoint resampling. Includes collision verification that falls back to unsmoothed A* if any waypoint clips an obstacle.
    6. **Dead-End / Blocker Stop**: If road is blocked ahead, returns a safe short path stopping $1.0\text{ m}$ before the nearest obstacle.
  - **Debug Export**: Exposes `planner.lastGrid` (and `lastGrid`) containing `cols`, `rows`, `resolution`, `originX`, `originY`, `data` (binary occupancy), and `costMap` (potential field) for Member D's 2D canvas and 3D overlay rendering.
  - **Performance**: Runs in **~2.4 ms** on average (well under the 15 ms target budget).

---

## 2. Shared Contract Compliance

Adheres strictly to [src/core/types.ts](file:///c:/Users/Venkat/Documents/HMTV/src/core/types.ts):
- World frame: $x$ = right, $y$ = forward (up on 2D canvas), origin at ego start.
- Heading $\theta = 0$ faces $+y$ (forward), positive counter-clockwise.
- Time step $dt = 1/60\text{ s}$.
- Units: meters and radians.
- Export signatures match exact specifications.

---

## 3. How to Run Standalone Tests

To run the headless verification suite:

```bash
# Run headless test suite with tsx
npx tsx src/planning/test_planner.ts

# Run strict type checking
npx tsc --noEmit
```

### Headless Test Scenarios Verified

1. **Baseline Straight**: Plans along $x \approx 0$ with $< 0.15\text{ m}$ deviation.
2. **Cow Avoidance**: Cow placed directly ahead at $(0, 15)\text{ m}$; planner visibly bends around it with $> 2.8\text{ m}$ lateral deviation.
3. **Cow Removal**: Path immediately straightens back to road centerline.
4. **Curve & Chaos Scenarios**: Robust path generation through complex curves and mixed traffic.
5. **Dynamic Stepping**: Validates `stepWorld` integration.
6. **Heatmap Debug Export**: Validates `planner.lastGrid` structure for Member D.
7. **Latency Benchmark**: 100 consecutive iterations verify average latency of $\approx 2.4\text{ ms}$ (P95 $< 3.5\text{ ms}$).

---

## 4. Teammate Integration Guide

- **Member A (`sim/controller.ts`, `sim/vehicle.ts`)**:
  Call `const path = plan(world, vehicleState);` and pass `path` into `pursuit(vehicleState, path)`.
- **Member C (`perception/replay.ts`)**:
  Recorded `World` frames can be fed directly to `plan(replayWorld, vehicleState)`.
- **Member D (`ui/hud.ts`, `view3d/scene3d.ts`)**:
  Access `planner.lastGrid` or `import { lastGrid } from './planning/planner'` to render the occupancy and clearance cost heatmap.
  Call `addObstacle(world, cls, x, y)` on canvas click to spawn dynamic obstacles interactively.
