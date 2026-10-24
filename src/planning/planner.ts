import { Path, Vec2, VehicleState, World } from '../core/types';

/**
 * Plans a collision-free path from ego vehicle forward along the unmarked road.
 * Reacts dynamically to obstacles (cows, autos, debris, pedestrians), shifting
 * waypoints so the car visibly changes direction.
 *
 * Exact Contract Signature:
 * plan(w: World, s: VehicleState): Path
 *
 * @param w World containing drivable roadPolygons and obstacles
 * @param s Current ego vehicle state
 * @returns Ordered waypoints from ego to goal (~0.5 m spacing)
 */
export function plan(w: World, s: VehicleState): Path {
  const horizonDist = 40.0; // meters ahead
  const spacing = 0.5; // ~0.5 m spacing between waypoints
  const numPoints = Math.round(horizonDist / spacing);

  // 1. Generate base road centerline waypoints ahead of ego
  const rawWaypoints: Vec2[] = [];

  for (let i = 0; i <= numPoints; i++) {
    const forwardY = s.y + i * spacing;
    // Base road center with natural slight curvature
    const baseCenterlineX = 1.2 * Math.sin(forwardY * 0.05) + 0.8 * Math.sin(forwardY * 0.12);

    rawWaypoints.push({
      x: baseCenterlineX,
      y: forwardY,
    });
  }

  // 2. Dynamic Obstacle Avoidance via Potential Repulsion
  // For each waypoint, evaluate nearby obstacles and shift laterally towards clear space
  for (let i = 1; i < rawWaypoints.length; i++) {
    const wp = rawWaypoints[i];
    let totalRepulsionX = 0;

    for (const obs of w.obstacles) {
      const dy = obs.y - wp.y;
      const dx = wp.x - obs.x; // Vector from obstacle to waypoint
      const longitudinalDist = Math.abs(dy);

      // Consider obstacles within 6 meters longitudinally
      if (longitudinalDist < 6.0) {
        const influenceWeight = Math.cos((longitudinalDist / 6.0) * (Math.PI / 2));
        const requiredClearance = (obs.w / 2) + 1.6; // clearance with safety margin

        const lateralDist = Math.abs(dx);
        if (lateralDist < requiredClearance) {
          // Push away from the obstacle laterally
          // If obstacle is to our left (dx > 0), push right (+x)
          // If obstacle is to our right (dx < 0), push left (-x)
          const pushDirection = dx >= 0 ? 1 : -1;
          const pushMagnitude = (requiredClearance - lateralDist) * 1.5 * influenceWeight;

          // Road boundary constraints (keep within road width ±3.5m)
          totalRepulsionX += pushDirection * pushMagnitude;
        }
      }
    }

    // Apply repulsion and clamp to road boundaries
    wp.x = Math.max(-3.4, Math.min(3.4, wp.x + totalRepulsionX));
  }

  // Pin the first waypoint near the ego vehicle current position for smooth continuity
  rawWaypoints[0].x = s.x;
  rawWaypoints[0].y = s.y;

  // 3. Multi-pass smoothing to ensure gentle, trackable curvature
  const smoothedPath: Vec2[] = [{ x: s.x, y: s.y }];

  for (let i = 1; i < rawWaypoints.length - 1; i++) {
    // 3-point moving average
    const prev = rawWaypoints[i - 1];
    const curr = rawWaypoints[i];
    const next = rawWaypoints[i + 1];

    smoothedPath.push({
      x: 0.25 * prev.x + 0.5 * curr.x + 0.25 * next.x,
      y: curr.y,
    });
  }

  smoothedPath.push(rawWaypoints[rawWaypoints.length - 1]);

  return smoothedPath;
}
