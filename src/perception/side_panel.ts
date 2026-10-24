// src/perception/side_panel.ts
// Side panel component: displays raw camera frame side-by-side with top-down 2D simulation view.
// Shows: Camera -> AI Perception -> Projected World Simulation
// SIH 2026 Autonomous Car Simulator - Member C

import type { World, Obstacle, Vec2, ObstacleClass, VehicleState } from '../core/types';
import { plan } from '../planning/planner';
import {
  type CameraConfig,
  DEFAULT_CAMERA_CONFIG,
  projectRawFrameToWorld,
  type RawFrameData
} from './projection';
import {
  getReplayWorld,
  getRawReplayFrame,
  getReplayFrameCount,
  isReplayLoaded
} from './replay';

export interface SidePanelOptions {
  container: HTMLElement;
  width?: number;
  height?: number;
  onFrameChange?: (frameIdx: number, world: World) => void;
}

const CLASS_COLORS: Record<ObstacleClass, { stroke: string; fill: string; text: string }> = {
  pedestrian: { stroke: '#ff0055', fill: 'rgba(255, 0, 85, 0.3)', text: '#ffffff' },
  two_wheeler: { stroke: '#ff9900', fill: 'rgba(255, 153, 0, 0.3)', text: '#ffffff' },
  auto: { stroke: '#ffd700', fill: 'rgba(255, 215, 0, 0.3)', text: '#000000' },
  car: { stroke: '#00ccff', fill: 'rgba(0, 204, 255, 0.3)', text: '#000000' },
  truck: { stroke: '#9933ff', fill: 'rgba(153, 51, 255, 0.3)', text: '#ffffff' },
  cow: { stroke: '#00ff88', fill: 'rgba(0, 255, 136, 0.3)', text: '#000000' },
  debris: { stroke: '#ff3333', fill: 'rgba(255, 51, 51, 0.3)', text: '#ffffff' }
};

export class PerceptionSidePanel {
  private container: HTMLElement;
  private currentFrame = 14; // 0-indexed (frame 15 is index 14)
  private isPlaying = false;
  private playIntervalId: number | null = null;
  private cameraConfig: CameraConfig = { ...DEFAULT_CAMERA_CONFIG };
  private onFrameChange?: (frameIdx: number, world: World) => void;

  // DOM elements
  private cameraCanvas!: HTMLCanvasElement;
  private cameraCtx!: CanvasRenderingContext2D;
  private worldCanvas!: HTMLCanvasElement;
  private worldCtx!: CanvasRenderingContext2D;

  private frameSlider!: HTMLInputElement;
  private frameLabel!: HTMLElement;
  private telemetryEl!: HTMLElement;
  private playBtn!: HTMLButtonElement;

  // Cached image element
  private activeImage = new Image();
  private imageCache = new Map<string, HTMLImageElement>();

  constructor(options: SidePanelOptions) {
    this.container = options.container;
    this.onFrameChange = options.onFrameChange;
    this.buildUI();
  }

  private buildUI(): void {
    this.container.innerHTML = `
      <div class="perception-side-panel" style="
        font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Roboto, sans-serif;
        background: #0d1117;
        color: #c9d1d9;
        border: 1px solid #30363d;
        border-radius: 8px;
        padding: 14px;
        box-shadow: 0 8px 24px rgba(0,0,0,0.5);
        max-width: 1200px;
        margin: 0 auto;
      ">
        <!-- Header -->
        <div style="display:flex; justify-content:space-between; align-items:center; border-bottom:1px solid #21262d; padding-bottom:10px; margin-bottom:12px;">
          <div>
            <span style="font-size:18px; font-weight:700; color:#58a6ff;">Perception & Replay Hub</span>
            <span style="font-size:12px; background:#238636; color:#fff; padding:2px 8px; border-radius:12px; margin-left:8px; font-weight:600;">DeepLabv3+ IDD-Lite</span>
          </div>
          <div id="perc-telemetry" style="font-size:13px; font-family:monospace; color:#8b949e;">
            Frame: 15 / 82 | Obstacles: 2 | Ego: (0.00m, 0.00m)
          </div>
        </div>

        <!-- Side-by-Side Dual Viewport -->
        <div style="display:grid; grid-template-columns: 1fr 1fr; gap: 14px; margin-bottom: 12px;">
          <!-- Camera Feed (Left) -->
          <div style="background:#161b22; border:1px solid #30363d; border-radius:6px; padding:8px;">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <span style="font-size:13px; font-weight:600; color:#f0f6fc;">1. Front Camera Feed (1080p Ego View)</span>
              <span style="font-size:11px; color:#58a6ff; font-family:monospace;">AI Segmentation Overlay</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9; background:#010409; border-radius:4px; overflow:hidden;">
              <canvas id="camera-canvas" width="640" height="360" style="width:100%; height:100%; object-fit:contain; display:block;"></canvas>
            </div>
          </div>

          <!-- Top-Down World View (Right) -->
          <div style="background:#161b22; border:1px solid #30363d; border-radius:6px; padding:8px;">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <span style="font-size:13px; font-weight:600; color:#f0f6fc;">2. Top-Down World Simulator (Meters)</span>
              <span style="font-size:11px; color:#3fb950; font-family:monospace;">Homography Projected (0..40m)</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9; background:#05090f; border-radius:4px; overflow:hidden;">
              <canvas id="world-canvas" width="640" height="360" style="width:100%; height:100%; object-fit:contain; display:block;"></canvas>
            </div>
          </div>
        </div>

        <!-- Controls Toolbar -->
        <div style="display:flex; flex-wrap:wrap; align-items:center; gap:12px; background:#161b22; padding:10px 14px; border-radius:6px; border:1px solid #30363d; margin-bottom:12px;">
          <button id="btn-play" style="background:#238636; color:#fff; border:none; padding:6px 14px; border-radius:6px; font-weight:600; cursor:pointer;">▶ Play</button>
          <button id="btn-prev" style="background:#21262d; color:#c9d1d9; border:1px solid #30363d; padding:6px 12px; border-radius:6px; cursor:pointer;">⏮ Prev</button>
          <button id="btn-next" style="background:#21262d; color:#c9d1d9; border:1px solid #30363d; padding:6px 12px; border-radius:6px; cursor:pointer;">Next ⏭</button>
          <button id="btn-frame15" style="background:#1f6feb; color:#fff; border:none; padding:6px 12px; border-radius:6px; font-weight:600; cursor:pointer;">Frame 15 (Judge Test)</button>

          <div style="display:flex; align-items:center; gap:8px; flex:1; min-width:200px;">
            <span style="font-size:12px; color:#8b949e;">Frame:</span>
            <input id="frame-slider" type="range" min="1" max="82" value="15" style="flex:1; cursor:pointer;">
            <span id="frame-counter" style="font-size:12px; font-family:monospace; color:#58a6ff; min-width:48px;">15/82</span>
          </div>
        </div>

        <!-- Homography Tuning Controls (Collapsible) -->
        <details style="background:#161b22; border:1px solid #30363d; border-radius:6px; padding:8px 12px;">
          <summary style="font-size:13px; font-weight:600; color:#58a6ff; cursor:pointer;">
            ⚙ Camera Homography Calibration & Constants
          </summary>
          <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap:12px; margin-top:10px;">
            <div>
              <label style="font-size:11px; color:#8b949e; display:block;">Camera Height: <span id="val-cam-height">1.40</span> m</label>
              <input id="tune-height" type="range" min="1.0" max="2.5" step="0.05" value="1.4" style="width:100%;">
            </div>
            <div>
              <label style="font-size:11px; color:#8b949e; display:block;">Horizon Ratio: <span id="val-horizon">0.32</span> (345.6 px)</label>
              <input id="tune-horizon" type="range" min="0.20" max="0.45" step="0.01" value="0.32" style="width:100%;">
            </div>
            <div>
              <label style="font-size:11px; color:#8b949e; display:block;">Focal Length fx/fy: <span id="val-fx">1000</span> px</label>
              <input id="tune-fx" type="range" min="600" max="1600" step="25" value="1000" style="width:100%;">
            </div>
            <div>
              <label style="font-size:11px; color:#8b949e; display:block;">Max Range Clip: <span id="val-range">40</span> m</label>
              <input id="tune-range" type="range" min="20" max="60" step="5" value="40" style="width:100%;">
            </div>
          </div>
        </details>
      </div>
    `;

    // Canvas references
    this.cameraCanvas = this.container.querySelector('#camera-canvas') as HTMLCanvasElement;
    this.cameraCtx = this.cameraCanvas.getContext('2d')!;
    this.worldCanvas = this.container.querySelector('#world-canvas') as HTMLCanvasElement;
    this.worldCtx = this.worldCanvas.getContext('2d')!;

    this.frameSlider = this.container.querySelector('#frame-slider') as HTMLInputElement;
    this.frameLabel = this.container.querySelector('#frame-counter') as HTMLElement;
    this.telemetryEl = this.container.querySelector('#perc-telemetry') as HTMLElement;
    this.playBtn = this.container.querySelector('#btn-play') as HTMLButtonElement;

    // Attach listeners
    this.frameSlider.addEventListener('input', (e) => {
      const idx = parseInt((e.target as HTMLInputElement).value, 10) - 1;
      this.goToFrame(idx);
    });

    this.playBtn.addEventListener('click', () => this.togglePlay());
    this.container.querySelector('#btn-prev')?.addEventListener('click', () => this.stepFrame(-1));
    this.container.querySelector('#btn-next')?.addEventListener('click', () => this.stepFrame(1));
    this.container.querySelector('#btn-frame15')?.addEventListener('click', () => this.goToFrame(14));

    // Tuning listeners
    this.setupTuningListeners();
  }

  private setupTuningListeners(): void {
    const heightInput = this.container.querySelector('#tune-height') as HTMLInputElement;
    const horizonInput = this.container.querySelector('#tune-horizon') as HTMLInputElement;
    const fxInput = this.container.querySelector('#tune-fx') as HTMLInputElement;
    const rangeInput = this.container.querySelector('#tune-range') as HTMLInputElement;

    heightInput?.addEventListener('input', (e) => {
      this.cameraConfig.height = parseFloat((e.target as HTMLInputElement).value);
      this.container.querySelector('#val-cam-height')!.textContent = this.cameraConfig.height.toFixed(2);
      this.render();
    });

    horizonInput?.addEventListener('input', (e) => {
      this.cameraConfig.horizonRatio = parseFloat((e.target as HTMLInputElement).value);
      const px = (this.cameraConfig.horizonRatio * 1080).toFixed(1);
      this.container.querySelector('#val-horizon')!.textContent = `${this.cameraConfig.horizonRatio.toFixed(2)} (${px}px)`;
      this.render();
    });

    fxInput?.addEventListener('input', (e) => {
      const val = parseFloat((e.target as HTMLInputElement).value);
      this.cameraConfig.fx = val;
      this.cameraConfig.fy = val;
      this.container.querySelector('#val-fx')!.textContent = val.toString();
      this.render();
    });

    rangeInput?.addEventListener('input', (e) => {
      this.cameraConfig.maxDistance = parseFloat((e.target as HTMLInputElement).value);
      this.container.querySelector('#val-range')!.textContent = this.cameraConfig.maxDistance.toString();
      this.render();
    });
  }

  public goToFrame(frameIdx: number): void {
    const total = getReplayFrameCount();
    if (total > 0) {
      this.currentFrame = Math.max(0, Math.min(frameIdx, total - 1));
      this.frameSlider.max = total.toString();
      this.frameSlider.value = (this.currentFrame + 1).toString();
      this.frameLabel.textContent = `${this.currentFrame + 1}/${total}`;
    } else {
      this.currentFrame = frameIdx;
    }

    this.render();

    if (this.onFrameChange) {
      const world = this.getCurrentWorld();
      this.onFrameChange(this.currentFrame, world);
    }
  }

  public stepFrame(delta: number): void {
    const total = getReplayFrameCount();
    if (total === 0) return;
    let next = this.currentFrame + delta;
    if (next < 0) next = total - 1;
    if (next >= total) next = 0;
    this.goToFrame(next);
  }

  public togglePlay(): void {
    this.isPlaying = !this.isPlaying;
    this.playBtn.textContent = this.isPlaying ? '⏸ Pause' : '▶ Play';
    this.playBtn.style.background = this.isPlaying ? '#d29922' : '#238636';

    if (this.isPlaying) {
      this.playIntervalId = window.setInterval(() => {
        this.stepFrame(1);
      }, 100); // 10 fps playback
    } else if (this.playIntervalId !== null) {
      clearInterval(this.playIntervalId);
      this.playIntervalId = null;
    }
  }

  private getCurrentWorld(): World {
    // Dynamically project current raw frame using active camera config
    const raw = getRawReplayFrame(this.currentFrame);
    if (raw) {
      return projectRawFrameToWorld(raw, this.cameraConfig);
    }
    return getReplayWorld(this.currentFrame);
  }

  /**
   * Renders both camera overlay and top-down world visualization.
   */
  public render(): void {
    const raw = getRawReplayFrame(this.currentFrame);
    const world = this.getCurrentWorld();

    // Update telemetry
    const total = getReplayFrameCount();
    const obsCount = world.obstacles.length;
    this.telemetryEl.textContent = `Frame: ${this.currentFrame + 1} / ${total || 82} | Obstacles: ${obsCount} | Time: ${world.t.toFixed(2)}s`;

    // 1. Render Left Canvas: Camera Feed
    this.renderCameraFeed(raw, world);

    // 2. Render Right Canvas: Top-Down World View
    this.renderTopDownWorld(world);
  }

  private renderCameraFeed(raw: RawFrameData | null, world: World): void {
    const ctx = this.cameraCtx;
    const w = this.cameraCanvas.width;
    const h = this.cameraCanvas.height;

    ctx.clearRect(0, 0, w, h);

    // Scale factors from 1920x1080 to canvas
    const sx = w / 1920;
    const sy = h / 1080;

    // Load and draw raw background frame if available
    const imgPath = raw?.image ? `/${raw.image}` : `/replay/frames/frame_${String(this.currentFrame + 1).padStart(3, '0')}.jpg`;
    let img = this.imageCache.get(imgPath);

    if (!img) {
      img = new Image();
      img.src = imgPath;
      img.onload = () => {
        this.imageCache.set(imgPath, img!);
        if (this.currentFrame === (raw?.frame ?? this.currentFrame + 1) - 1) {
          this.render();
        }
      };
    }

    if (img && img.complete && img.naturalWidth > 0) {
      ctx.drawImage(img, 0, 0, w, h);
    } else {
      // Dark asphalt backdrop fallback
      ctx.fillStyle = '#1c2128';
      ctx.fillRect(0, 0, w, h);
      ctx.fillStyle = '#484f58';
      ctx.font = '14px monospace';
      ctx.fillText(`Loading Frame ${this.currentFrame + 1}...`, w / 2 - 80, h / 2);
    }

    // Draw Horizon Line
    const horizonY = this.cameraConfig.horizonRatio * 1080 * sy;
    ctx.strokeStyle = 'rgba(255, 200, 0, 0.6)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, horizonY);
    ctx.lineTo(w, horizonY);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = 'rgba(255, 200, 0, 0.8)';
    ctx.font = '10px monospace';
    ctx.fillText(`Horizon: ${(this.cameraConfig.horizonRatio * 1080).toFixed(0)}px`, 8, horizonY - 4);

    if (!raw) return;

    // Draw Road Polygon overlay in camera view
    if (raw.road && raw.road.length > 0) {
      ctx.fillStyle = 'rgba(0, 255, 128, 0.22)';
      ctx.strokeStyle = '#00ff80';
      ctx.lineWidth = 2;
      ctx.beginPath();

      const roadPts = Array.isArray(raw.road[0]) && Array.isArray((raw.road[0] as unknown[])[0])
        ? (raw.road[0] as [number, number][])
        : (raw.road as [number, number][]);

      roadPts.forEach((pt, i) => {
        const px = (Array.isArray(pt) ? pt[0] : (pt as unknown as Vec2).x) * sx;
        const py = (Array.isArray(pt) ? pt[1] : (pt as unknown as Vec2).y) * sy;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }

    // Draw Obstacle Bounding Boxes in camera view
    if (raw.obstacles) {
      raw.obstacles.forEach((obs, idx) => {
        const [x1, y1, x2, y2] = obs.bbox;
        const bx = x1 * sx;
        const by = y1 * sy;
        const bw = (x2 - x1) * sx;
        const bh = (y2 - y1) * sy;

        const cls = (obs.cls || 'car') as ObstacleClass;
        const color = CLASS_COLORS[cls] || CLASS_COLORS.car;

        ctx.strokeStyle = color.stroke;
        ctx.lineWidth = 2;
        ctx.strokeRect(bx, by, bw, bh);

        // Find corresponding projected world distance
        const worldObs = world.obstacles[idx];
        const distStr = worldObs ? `${worldObs.y.toFixed(1)}m` : '';

        ctx.fillStyle = color.stroke;
        const labelText = `${cls.toUpperCase()} ${distStr}`;
        ctx.font = 'bold 11px sans-serif';
        const textWidth = ctx.measureText(labelText).width;
        ctx.fillRect(bx, Math.max(0, by - 16), textWidth + 6, 16);

        ctx.fillStyle = color.text;
        ctx.fillText(labelText, bx + 3, Math.max(12, by - 4));
      });
    }
  }

  private renderTopDownWorld(world: World): void {
    const ctx = this.worldCtx;
    const w = this.worldCanvas.width;
    const h = this.worldCanvas.height;

    ctx.clearRect(0, 0, w, h);

    // World to Canvas transform:
    // World coordinates: x = right (meters), y = forward (meters, 0 to 40m)
    // Canvas: ego origin at (w / 2, h - 30px)
    // Scale: e.g. 7.5 pixels per meter gives 40m * 7.5 = 300px vertical range
    const originX = w / 2;
    const originY = h - 35;
    const scale = 7.5; // px / meter

    const toCanvasX = (worldX: number) => originX + worldX * scale;
    const toCanvasY = (worldY: number) => originY - worldY * scale;

    // Draw Distance Grid & Arcs
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;

    [10, 20, 30, 40].forEach((dist) => {
      const cy = toCanvasY(dist);
      ctx.beginPath();
      ctx.moveTo(30, cy);
      ctx.lineTo(w - 30, cy);
      ctx.stroke();

      ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.font = '9px monospace';
      ctx.fillText(`${dist}m`, originX + 5, cy - 3);

      // Radial range arc
      ctx.beginPath();
      ctx.arc(originX, originY, dist * scale, -Math.PI, 0);
      ctx.stroke();
    });

    // Lateral grid lines (-10m, -5m, +5m, +10m)
    [-10, -5, 5, 10].forEach((lat) => {
      const cx = toCanvasX(lat);
      ctx.beginPath();
      ctx.moveTo(cx, 10);
      ctx.lineTo(cx, originY);
      ctx.stroke();

      ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
      ctx.font = '9px monospace';
      ctx.fillText(`${lat > 0 ? '+' : ''}${lat}m`, cx - 8, originY + 14);
    });

    // Draw Road Polygons in World Space
    if (world.roadPolygons && world.roadPolygons.length > 0) {
      for (const poly of world.roadPolygons) {
        if (poly.length < 3) continue;

        ctx.fillStyle = 'rgba(40, 70, 95, 0.4)';
        ctx.strokeStyle = '#388bfd';
        ctx.lineWidth = 2;
        ctx.beginPath();

        poly.forEach((pt, i) => {
          const cx = toCanvasX(pt.x);
          const cy = toCanvasY(pt.y);
          if (i === 0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        });

        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      }
    }

    // Draw Planned Trajectory from Member B (A* Path Planner)
    try {
      const egoState: VehicleState = { x: 0, y: 0, theta: 0, v: 5, delta: 0 };
      const path = plan(world, egoState);
      if (path && path.length > 1) {
        ctx.save();
        ctx.strokeStyle = '#00f0ff';
        ctx.lineWidth = 3;
        ctx.shadowColor = '#00f0ff';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        path.forEach((pt, i) => {
          const px = toCanvasX(pt.x);
          const py = toCanvasY(pt.y);
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        });
        ctx.stroke();
        ctx.restore();

        // Waypoint nodes along the path
        ctx.fillStyle = '#ffffff';
        path.forEach((pt, i) => {
          if (i % 3 === 0) {
            const px = toCanvasX(pt.x);
            const py = toCanvasY(pt.y);
            ctx.beginPath();
            ctx.arc(px, py, 2, 0, Math.PI * 2);
            ctx.fill();
          }
        });
      }
    } catch {
      // Graceful fallback if planner cannot find path
    }

    // Draw Ego Vehicle at origin (0, 0)
    this.renderEgoVehicle(ctx, originX, originY, scale);

    // Draw Projected Obstacles
    world.obstacles.forEach((obs) => {
      this.renderWorldObstacle(ctx, obs, toCanvasX, toCanvasY, scale);
    });
  }

  private renderEgoVehicle(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    scale: number
  ): void {
    const egoW = 1.9 * scale;
    const egoL = 4.4 * scale;

    ctx.save();
    ctx.translate(cx, cy);

    // Shadow
    ctx.fillStyle = 'rgba(0,0,0,0.4)';
    ctx.fillRect(-egoW / 2 + 2, -egoL + 2, egoW, egoL);

    // Body
    ctx.fillStyle = '#238636';
    ctx.strokeStyle = '#3fb950';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-egoW / 2, -egoL, egoW, egoL, 4);
    ctx.fill();
    ctx.stroke();

    // Windshield
    ctx.fillStyle = '#161b22';
    ctx.fillRect(-egoW / 2 + 3, -egoL + 8, egoW - 6, egoL * 0.25);

    // Headlights
    ctx.fillStyle = '#f0f6fc';
    ctx.fillRect(-egoW / 2 + 2, -egoL, 4, 3);
    ctx.fillRect(egoW / 2 - 6, -egoL, 4, 3);

    // Ego Center Marker
    ctx.fillStyle = '#58a6ff';
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
  }

  private renderWorldObstacle(
    ctx: CanvasRenderingContext2D,
    obs: Obstacle,
    toCanvasX: (x: number) => number,
    toCanvasY: (y: number) => number,
    scale: number
  ): void {
    const cx = toCanvasX(obs.x);
    const cy = toCanvasY(obs.y);
    const w = obs.w * scale;
    const l = obs.l * scale;

    const color = CLASS_COLORS[obs.cls] || CLASS_COLORS.car;

    ctx.save();
    ctx.translate(cx, cy);
    // Heading: 0 means facing +y (forward / up on canvas).
    // In canvas coords, y is inverted, so heading theta tilts clockwise by -theta
    ctx.rotate(-obs.heading);

    // Obstacle Box
    ctx.fillStyle = color.fill;
    ctx.strokeStyle = color.stroke;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.roundRect(-w / 2, -l / 2, w, l, 2);
    ctx.fill();
    ctx.stroke();

    // Front orientation indicator
    ctx.fillStyle = color.stroke;
    ctx.fillRect(-w / 2, -l / 2, w, 2);

    ctx.restore();

    // Velocity Vector Arrow
    if (Math.hypot(obs.vx, obs.vy) > 0.2) {
      const vScale = 1.2; // seconds of lookahead
      const vxEnd = toCanvasX(obs.x + obs.vx * vScale);
      const vyEnd = toCanvasY(obs.y + obs.vy * vScale);

      ctx.strokeStyle = '#ffd700';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(vxEnd, vyEnd);
      ctx.stroke();

      // Arrow head
      ctx.fillStyle = '#ffd700';
      ctx.beginPath();
      ctx.arc(vxEnd, vyEnd, 3, 0, Math.PI * 2);
      ctx.fill();
    }

    // Label with Class and Coordinates
    ctx.fillStyle = color.stroke;
    ctx.font = 'bold 10px monospace';
    const tag = `${obs.cls.toUpperCase()} (${obs.x > 0 ? '+' : ''}${obs.x.toFixed(1)}m, ${obs.y.toFixed(1)}m)`;
    ctx.fillText(tag, cx + w / 2 + 4, cy);
  }

  public getCameraConfig(): CameraConfig {
    return { ...this.cameraConfig };
  }

  public setCameraConfig(cfg: Partial<CameraConfig>): void {
    this.cameraConfig = { ...this.cameraConfig, ...cfg };
    this.render();
  }
}
