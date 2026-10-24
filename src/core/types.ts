// Shared Contract: src/core/types.ts
// Source of truth created by Member A for SIH 2026 Autonomous Vehicle Simulator

export type Vec2 = { x: number; y: number };

export type ObstacleClass =
  | 'pedestrian'
  | 'two_wheeler'
  | 'auto'
  | 'car'
  | 'truck'
  | 'cow'
  | 'debris';

export interface Obstacle {
  id: string;
  cls: ObstacleClass;
  x: number;
  y: number;
  w: number;
  l: number;
  heading: number;
  vx: number;
  vy: number;
}

export interface VehicleState {
  x: number;
  y: number;
  theta: number; // 0 = facing +y (forward), positive = counter-clockwise
  v: number;
  delta: number;
}

export interface ControlCmd {
  steer: number; // rad, clamp ±0.6
  accel: number; // m/s², clamp -6..3
}

export interface World {
  roadPolygons: Vec2[][]; // drivable area, no lane lines
  obstacles: Obstacle[];
  t: number;
}

export type Path = Vec2[]; // ordered waypoints from ego to goal, ~0.5 m spacing
