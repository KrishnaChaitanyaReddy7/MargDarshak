import { Obstacle, Vec2, World } from '../core/types';

/**
 * Creates an unmarked Indian road scenario with irregular shoulders,
 * dense mixed traffic (cows, auto-rickshaws, pedestrians, two-wheelers, debris, trucks).
 *
 * @param scenario Scenario selector ('dense_mixed' | 'bottleneck' | 'moving_traffic' or '1'|'2'|'3')
 */
export function createWorld(scenario: string = '1'): World {
  // Road polygon: Unmarked Indian road (~8 to 11 meters wide, curvy shoulders, 150m long)
  // Left shoulder boundary (x < 0) and right shoulder boundary (x > 0)
  const leftBoundary: Vec2[] = [];
  const rightBoundary: Vec2[] = [];

  for (let y = -10; y <= 160; y += 4) {
    // Natural slight curves and irregular shoulder variations
    const roadCenter = 1.2 * Math.sin(y * 0.05) + 0.8 * Math.sin(y * 0.12);
    const halfWidth = 4.2 + 0.6 * Math.sin(y * 0.08); // 8.4m to 9.6m wide
    leftBoundary.push({ x: roadCenter - halfWidth, y });
    rightBoundary.push({ x: roadCenter + halfWidth, y });
  }

  // Polygon formed by walking up right shoulder then down left shoulder
  const roadPolygon: Vec2[] = [...rightBoundary, ...leftBoundary.reverse()];

  let obstacles: Obstacle[] = [];

  switch (scenario) {
    case '2':
    case 'bottleneck':
      // Narrow bottleneck with debris, parked auto, and pedestrians
      obstacles = [
        {
          id: 'debris-1',
          cls: 'debris',
          x: -1.2,
          y: 18.0,
          w: 1.2,
          l: 1.0,
          heading: 0.1,
          vx: 0,
          vy: 0,
        },
        {
          id: 'auto-parked',
          cls: 'auto',
          x: 2.2,
          y: 28.0,
          w: 1.5,
          l: 2.8,
          heading: -0.05,
          vx: 0,
          vy: 0,
        },
        {
          id: 'pedestrian-cross',
          cls: 'pedestrian',
          x: -2.5,
          y: 42.0,
          w: 0.6,
          l: 0.6,
          heading: -Math.PI / 2,
          vx: 0.8,
          vy: 0.0,
        },
        {
          id: 'cow-1',
          cls: 'cow',
          x: 1.0,
          y: 65.0,
          w: 1.1,
          l: 2.2,
          heading: 0.4,
          vx: 0,
          vy: 0,
        },
        {
          id: 'truck-ahead',
          cls: 'truck',
          x: -0.5,
          y: 95.0,
          w: 2.4,
          l: 7.0,
          heading: 0.0,
          vx: 0.0,
          vy: 2.2,
        },
      ];
      break;

    case '3':
    case 'moving_traffic':
      // Dynamic scenario with overtaking auto, weaving two-wheeler, oncoming traffic
      obstacles = [
        {
          id: 'two-wheeler-1',
          cls: 'two_wheeler',
          x: -1.5,
          y: 15.0,
          w: 0.8,
          l: 1.8,
          heading: 0.05,
          vx: 0.15,
          vy: 4.5,
        },
        {
          id: 'auto-ahead',
          cls: 'auto',
          x: 0.8,
          y: 30.0,
          w: 1.5,
          l: 2.8,
          heading: 0.0,
          vx: -0.1,
          vy: 3.5,
        },
        {
          id: 'cow-roadside',
          cls: 'cow',
          x: -2.8,
          y: 50.0,
          w: 1.1,
          l: 2.3,
          heading: 0.8,
          vx: 0,
          vy: 0,
        },
        {
          id: 'car-ahead',
          cls: 'car',
          x: -0.8,
          y: 75.0,
          w: 1.8,
          l: 4.4,
          heading: 0.0,
          vx: 0.0,
          vy: 4.0,
        },
        {
          id: 'pedestrian-jaywalker',
          cls: 'pedestrian',
          x: 2.2,
          y: 92.0,
          w: 0.6,
          l: 0.6,
          heading: Math.PI / 2,
          vx: -0.6,
          vy: 0.0,
        },
      ];
      break;

    case '1':
    case 'dense_mixed':
    default:
      // Canonical SIH dense Indian road scenario: Holy cow in middle, auto-rickshaws, pedestrians
      obstacles = [
        {
          id: 'cow-sacred',
          cls: 'cow',
          x: 0.2,
          y: 22.0,
          w: 1.2,
          l: 2.4,
          heading: 0.3,
          vx: 0.0,
          vy: 0.0,
        },
        {
          id: 'auto-rickshaw-1',
          cls: 'auto',
          x: -1.8,
          y: 36.0,
          w: 1.5,
          l: 2.8,
          heading: 0.0,
          vx: 0.05,
          vy: 3.2,
        },
        {
          id: 'pedestrian-1',
          cls: 'pedestrian',
          x: 2.6,
          y: 45.0,
          w: 0.6,
          l: 0.6,
          heading: Math.PI / 2,
          vx: -0.7,
          vy: 0.1,
        },
        {
          id: 'two-wheeler-fast',
          cls: 'two_wheeler',
          x: 1.5,
          y: 58.0,
          w: 0.8,
          l: 1.8,
          heading: -0.05,
          vx: -0.1,
          vy: 4.8,
        },
        {
          id: 'debris-pothole',
          cls: 'debris',
          x: -0.6,
          y: 72.0,
          w: 1.4,
          l: 1.2,
          heading: 0.0,
          vx: 0.0,
          vy: 0.0,
        },
        {
          id: 'truck-slow',
          cls: 'truck',
          x: 1.2,
          y: 95.0,
          w: 2.4,
          l: 7.2,
          heading: 0.0,
          vx: 0.0,
          vy: 2.5,
        },
        {
          id: 'car-cruising',
          cls: 'car',
          x: -1.6,
          y: 120.0,
          w: 1.8,
          l: 4.3,
          heading: 0.0,
          vx: 0.0,
          vy: 4.0,
        },
      ];
      break;
  }

  return {
    roadPolygons: [roadPolygon],
    obstacles,
    t: 0,
  };
}

/**
 * Steps the world simulation forward by dt seconds.
 * Moves dynamic obstacles and wraps or bounds them within the road corridor.
 */
export function stepWorld(w: World, dt: number): void {
  w.t += dt;

  for (const obs of w.obstacles) {
    obs.x += obs.vx * dt;
    obs.y += obs.vy * dt;

    // Pedestrian bounce off road shoulders
    if (obs.cls === 'pedestrian') {
      if (obs.x > 3.8 && obs.vx > 0) obs.vx = -Math.abs(obs.vx);
      if (obs.x < -3.8 && obs.vx < 0) obs.vx = Math.abs(obs.vx);
    }

    // Moving vehicles gentle lateral wandering to simulate realistic Indian driving
    if (obs.cls === 'auto' || obs.cls === 'two_wheeler') {
      if (Math.abs(obs.vx) > 0.01) {
        if (obs.x > 3.0 && obs.vx > 0) obs.vx = -obs.vx;
        if (obs.x < -3.0 && obs.vx < 0) obs.vx = -obs.vx;
      }
    }
  }
}
