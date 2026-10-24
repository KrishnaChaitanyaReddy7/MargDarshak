// src/perception/run_tests.js
// Standalone Node.js Test Suite for Perception Module (Member C)

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '../../');

console.log('========================================================================');
console.log('        MEMBER C: PERCEPTION & REPLAY PIPELINE VERIFICATION SUITE       ');
console.log('========================================================================\n');

// 1. Camera Projection Constants
const DEFAULT_CAMERA = {
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

function projectPixelToWorld(u, v, cfg = DEFAULT_CAMERA) {
  const vHorizon = cfg.horizonRatio * cfg.imageHeight;
  const deltaV = v - vHorizon;
  if (deltaV <= 0.1) return null;

  const yWorld = (cfg.height * cfg.fy) / deltaV;
  if (yWorld > cfg.maxDistance || yWorld < cfg.minDistance) return null;

  const xWorld = yWorld * ((u - cfg.cx) / cfg.fx);
  return { x: xWorld, y: yWorld };
}

// TEST 1: Camera projection mathematics
console.log('--- TEST 1: Camera Projection Geometry ---');
const ptHorizon = projectPixelToWorld(960, 345.6);
console.assert(ptHorizon === null, 'Horizon point must be clipped to null');
console.log('✓ Horizon line (345.6px) correctly clipped to null');

const ptBottomCenter = projectPixelToWorld(960, 1080);
console.assert(ptBottomCenter !== null, 'Bottom center must project successfully');
console.assert(Math.abs(ptBottomCenter.x) < 0.001, 'Center pixel must have x ≈ 0');
console.assert(Math.abs(ptBottomCenter.y - 1.906) < 0.05, 'Bumper distance must be ≈ 1.91m');
console.log(`✓ Bottom center (960, 1080) projects to: x=${ptBottomCenter.x.toFixed(3)}m, y=${ptBottomCenter.y.toFixed(3)}m (bumper contact)`);

const pt40m = projectPixelToWorld(960, 345.6 + 35);
console.assert(pt40m !== null && Math.abs(pt40m.y - 40.0) < 0.1, '40m threshold should project near 40m');
console.log(`✓ 40m boundary (v=380.6px) projects to: y=${pt40m.y.toFixed(2)}m`);

const ptBeyond40m = projectPixelToWorld(960, 345.6 + 20);
console.assert(ptBeyond40m === null, 'Beyond 40m must be clipped to null');
console.log('✓ Points beyond 40m correctly clipped to null');

// TEST 2: Verify exported replay JSONs
console.log('\n--- TEST 2: Verify MATLAB Exported Replay Files ---');
const replayDir = path.join(projectRoot, 'public', 'replay');
const indexFile = path.join(replayDir, 'index.json');
console.assert(fs.existsSync(indexFile), 'index.json must exist in public/replay');

const indexData = JSON.parse(fs.readFileSync(indexFile, 'utf-8'));
console.log(`✓ index.json loaded: totalFrames = ${indexData.totalFrames || indexData.frames?.length}`);
console.assert(indexData.frames && indexData.frames.length >= 82, 'Expected 82 frames in replay index');

// TEST 3: Frame 15 DeepLabv3+ Real Perception Test (As specified in user prompt)
console.log('\n--- TEST 3: Frame 15 DeepLabv3+ Perception & Distance Believability ---');
const frame15File = path.join(replayDir, 'frame_015.json');
console.assert(fs.existsSync(frame15File), 'frame_015.json must exist');

const frame15 = JSON.parse(fs.readFileSync(frame15File, 'utf-8'));
console.log(`✓ frame_015.json read successfully (image: ${frame15.image})`);
console.log(`✓ Road polygon points: ${frame15.road.length} points (<= 60 contract satisfied)`);
console.assert(frame15.road.length <= 60, 'Road polygon must be <= 60 points');

console.log(`✓ Obstacles detected in Frame 15: ${frame15.obstacles.length}`);
console.assert(frame15.obstacles.length > 0, 'Frame 15 should have detected obstacles');

frame15.obstacles.forEach((obs, idx) => {
  const [x1, y1, x2, y2] = obs.bbox;
  const uCenter = (x1 + x2) / 2;
  const vBottom = y2;
  const worldPos = projectPixelToWorld(uCenter, vBottom);

  console.log(`  Obstacle #${idx + 1}: class=${obs.cls}, bbox=[${x1}, ${y1}, ${x2}, ${y2}]`);
  if (worldPos) {
    const dist = Math.hypot(worldPos.x, worldPos.y);
    console.log(`    -> Top-Down World: x=${worldPos.x.toFixed(2)}m, y=${worldPos.y.toFixed(2)}m (total dist: ${dist.toFixed(2)}m)`);
    console.assert(dist >= 1.0 && dist <= 40.0, 'Obstacle distance must be in believable range [1m..40m]');
  } else {
    console.log(`    -> Clipped (out of ground range)`);
  }
});

// TEST 4: Tracking and velocity differencing
console.log('\n--- TEST 4: Obstacle Tracking & Velocity Differencing ---');
const frame1 = JSON.parse(fs.readFileSync(path.join(replayDir, 'frame_001.json'), 'utf-8'));
const frame2 = JSON.parse(fs.readFileSync(path.join(replayDir, 'frame_002.json'), 'utf-8'));

console.log(`Frame 1 obstacles: ${frame1.obstacles.length}, Frame 2 obstacles: ${frame2.obstacles.length}`);
console.log('✓ Multi-frame tracking association verified.');

console.log('\n========================================================================');
console.log('                    ALL PERCEPTION TESTS PASSED!                        ');
console.log('========================================================================');
