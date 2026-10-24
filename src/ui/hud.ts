import { Obstacle, Path, VehicleState, World } from '../core/types';
import { EGO_LENGTH, EGO_WIDTH, evaluateCollisions } from '../sim/collision';

/**
 * 2D Canvas Renderer and Heads-Up Display (HUD) for Unmarked Indian Road Simulation.
 *
 * Exact Contract Signature:
 * render2D(ctx: CanvasRenderingContext2D, s: VehicleState, w: World, p: Path): void
 */
export function render2D(
  ctx: CanvasRenderingContext2D,
  s: VehicleState,
  w: World,
  p: Path
): void {
  const canvas = ctx.canvas;
  const width = canvas.width;
  const height = canvas.height;

  // Clear background (dirt / shoulder terrain color)
  ctx.fillStyle = '#1c1b18';
  ctx.fillRect(0, 0, width, height);

  // Camera Settings: Scale (pixels per meter), centered on ego vehicle
  const scale = 22.0; // 22 pixels per meter
  const camX = s.x;
  const camY = s.y + 12.0; // Offset camera slightly ahead so driver sees more forward road

  const originX = width / 2;
  const originY = height * 0.72;

  // World to screen transform helper
  const toScreen = (wx: number, wy: number) => {
    return {
      sx: originX + (wx - camX) * scale,
      sy: originY - (wy - camY) * scale, // Canvas Y is inverted (up is -y)
    };
  };

  // 1. Draw Drivable Road Polygon (Unmarked road with dusty shoulders)
  if (w.roadPolygons && w.roadPolygons.length > 0) {
    for (const poly of w.roadPolygons) {
      if (poly.length === 0) continue;

      // Draw road asphalt
      ctx.beginPath();
      const first = toScreen(poly[0].x, poly[0].y);
      ctx.moveTo(first.sx, first.sy);
      for (let i = 1; i < poly.length; i++) {
        const pt = toScreen(poly[i].x, poly[i].y);
        ctx.lineTo(pt.sx, pt.sy);
      }
      ctx.closePath();

      ctx.fillStyle = '#2b2d35'; // Dark worn asphalt
      ctx.fill();

      // Irregular shoulder boundary lines (dusty ochre)
      ctx.strokeStyle = '#6e624c';
      ctx.lineWidth = 3;
      ctx.stroke();
    }
  }

  // 2. Draw Planned Autopilot Path
  if (p && p.length > 1) {
    // Path glow
    ctx.beginPath();
    const startP = toScreen(p[0].x, p[0].y);
    ctx.moveTo(startP.sx, startP.sy);
    for (let i = 1; i < p.length; i++) {
      const pt = toScreen(p[i].x, p[i].y);
      ctx.lineTo(pt.sx, pt.sy);
    }
    ctx.strokeStyle = 'rgba(0, 255, 170, 0.25)';
    ctx.lineWidth = 6;
    ctx.stroke();

    // Sharp central trajectory
    ctx.beginPath();
    ctx.moveTo(startP.sx, startP.sy);
    for (let i = 1; i < p.length; i++) {
      const pt = toScreen(p[i].x, p[i].y);
      ctx.lineTo(pt.sx, pt.sy);
    }
    ctx.strokeStyle = '#00ffaa';
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Waypoint dots
    ctx.fillStyle = '#afffd9';
    for (let i = 0; i < p.length; i += 3) {
      const pt = toScreen(p[i].x, p[i].y);
      ctx.beginPath();
      ctx.arc(pt.sx, pt.sy, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // 3. Draw Obstacles (Indian traffic: cows, auto-rickshaws, two-wheelers, pedestrians, debris)
  for (const obs of w.obstacles) {
    const screenCenter = toScreen(obs.x, obs.y);

    ctx.save();
    ctx.translate(screenCenter.sx, screenCenter.sy);
    // Heading in world: 0 is +y (up), CCW is positive
    // Canvas rotation: 0 is +x, CW is positive
    // So world angle theta maps to canvas rotation: -theta
    ctx.rotate(-obs.heading);

    const wPx = obs.w * scale;
    const lPx = obs.l * scale;

    // Obstacle color styling based on class
    let bodyColor = '#f39c12';
    let label = obs.cls.toUpperCase();

    switch (obs.cls) {
      case 'cow':
        bodyColor = '#e3b070'; // Cow beige/tan
        label = 'COW';
        break;
      case 'auto':
        bodyColor = '#ffd000'; // Bajaj auto yellow
        label = 'AUTO';
        break;
      case 'two_wheeler':
        bodyColor = '#d946ef'; // Scooter magenta
        label = '2-WHEELER';
        break;
      case 'pedestrian':
        bodyColor = '#38bdf8'; // Pedestrian cyan
        label = 'PED';
        break;
      case 'debris':
        bodyColor = '#64748b'; // Slate pothole/rock
        label = 'DEBRIS';
        break;
      case 'truck':
        bodyColor = '#ea580c'; // Indian Tata truck orange
        label = 'TRUCK';
        break;
      case 'car':
        bodyColor = '#3b82f6';
        label = 'CAR';
        break;
    }

    // Body rectangle
    ctx.fillStyle = bodyColor;
    ctx.beginPath();
    ctx.roundRect(-wPx / 2, -lPx / 2, wPx, lPx, 4);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Direction arrow on top of obstacle
    ctx.fillStyle = '#111827';
    ctx.beginPath();
    ctx.moveTo(0, -lPx / 2 + 3);
    ctx.lineTo(-wPx / 4, -lPx / 4);
    ctx.lineTo(wPx / 4, -lPx / 4);
    ctx.closePath();
    ctx.fill();

    // Label tag
    ctx.font = 'bold 9px monospace';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText(label, 0, lPx / 2 + 11);

    ctx.restore();
  }

  // 4. Draw Ego Vehicle (1.8m x 4.2m)
  const egoScreen = toScreen(s.x, s.y);
  ctx.save();
  ctx.translate(egoScreen.sx, egoScreen.sy);
  ctx.rotate(-s.theta);

  const egoWPx = EGO_WIDTH * scale;
  const egoLPx = EGO_LENGTH * scale;

  // Collision highlight
  const colEval = evaluateCollisions(s, w.obstacles);
  if (colEval.isColliding) {
    ctx.fillStyle = 'rgba(255, 0, 0, 0.4)';
    ctx.beginPath();
    ctx.arc(0, 0, egoLPx, 0, Math.PI * 2);
    ctx.fill();
  }

  // Vehicle shadow
  ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
  ctx.beginPath();
  ctx.roundRect(-egoWPx / 2 + 2, -egoLPx / 2 + 3, egoWPx, egoLPx, 6);
  ctx.fill();

  // Vehicle Body
  ctx.fillStyle = colEval.isColliding ? '#ef4444' : '#00e5ff';
  ctx.beginPath();
  ctx.roundRect(-egoWPx / 2, -egoLPx / 2, egoWPx, egoLPx, 6);
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Roof / Windshield
  ctx.fillStyle = '#0f172a';
  ctx.beginPath();
  ctx.roundRect(-egoWPx * 0.35, -egoLPx * 0.25, egoWPx * 0.7, egoLPx * 0.45, 4);
  ctx.fill();

  // Headlights
  ctx.fillStyle = '#fef08a';
  ctx.fillRect(-egoWPx / 2 + 3, -egoLPx / 2 - 2, 7, 3);
  ctx.fillRect(egoWPx / 2 - 10, -egoLPx / 2 - 2, 7, 3);

  // Steered Front Wheels
  const wheelW = 4;
  const wheelL = 12;
  const frontAxleY = -egoLPx * 0.32;
  const rearAxleY = egoLPx * 0.32;

  // Front left wheel
  ctx.save();
  ctx.translate(-egoWPx / 2, frontAxleY);
  ctx.rotate(-s.delta); // Steering angle
  ctx.fillStyle = '#1e293b';
  ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
  ctx.restore();

  // Front right wheel
  ctx.save();
  ctx.translate(egoWPx / 2, frontAxleY);
  ctx.rotate(-s.delta); // Steering angle
  ctx.fillStyle = '#1e293b';
  ctx.fillRect(-wheelW / 2, -wheelL / 2, wheelW, wheelL);
  ctx.restore();

  // Rear wheels (fixed)
  ctx.fillStyle = '#1e293b';
  ctx.fillRect(-egoWPx / 2 - wheelW / 2, rearAxleY - wheelL / 2, wheelW, wheelL);
  ctx.fillRect(egoWPx / 2 - wheelW / 2, rearAxleY - wheelL / 2, wheelW, wheelL);

  ctx.restore();

  // 5. HUD Telemetry Card Overlay (Top-Left)
  ctx.save();
  ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(14, 14, 270, 195, 8);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = '#38bdf8';
  ctx.font = 'bold 13px system-ui, sans-serif';
  ctx.fillText('SIH 2026 UNMARKED ROAD SIM', 26, 36);

  ctx.font = '11px monospace';
  ctx.fillStyle = '#e2e8f0';

  const speedKmph = (s.v * 3.6).toFixed(1);
  const steerDeg = ((s.delta * 180) / Math.PI).toFixed(1);

  ctx.fillText(`SPEED:     ${speedKmph.padStart(5, ' ')} km/h (${s.v.toFixed(2)} m/s)`, 26, 60);
  ctx.fillText(`STEERING:  ${steerDeg.padStart(5, ' ')}° (${s.delta.toFixed(2)} rad)`, 26, 80);
  ctx.fillText(`HEADING:   ${((s.theta * 180) / Math.PI).toFixed(1)}°`, 26, 100);
  ctx.fillText(`POS (X,Y): (${s.x.toFixed(1)}, ${s.y.toFixed(1)}) m`, 26, 120);

  // Obstacle warning
  if (colEval.minObstacleDistance < 8.0) {
    ctx.fillStyle = colEval.minObstacleDistance < 3.0 ? '#ef4444' : '#f59e0b';
    ctx.fillText(`NEAREST OBS: ${colEval.minObstacleDistance.toFixed(1)} m [ALERT]`, 26, 142);
  } else {
    ctx.fillStyle = '#10b981';
    ctx.fillText(`NEAREST OBS: ${colEval.minObstacleDistance.toFixed(1)} m [CLEAR]`, 26, 142);
  }

  // Collision counter
  ctx.fillStyle = colEval.isColliding ? '#ef4444' : '#94a3b8';
  ctx.fillText(`COLLISION:  ${colEval.isColliding ? 'IMPACT DETECTED!' : 'NONE'}`, 26, 164);

  // Status pills
  ctx.fillStyle = '#64748b';
  ctx.font = '10px system-ui, sans-serif';
  ctx.fillText(`[M] Mode: Autopilot/Manual  [Space] Pause  [R] Reset  [1-3] Scenarios`, 26, 192);

  ctx.restore();
}
