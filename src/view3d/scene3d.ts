import { Path, VehicleState, World } from '../core/types';

/**
 * Minimal compile-safe stub for Member D (3D Visualization).
 * Matches exact shared contract:
 * mount(el: HTMLElement): void
 */
export function mount(el: HTMLElement): void {
  el.innerHTML = `
    <div style="display:flex;flex-direction:column;height:100%;align-items:center;justify-content:center;color:#94a3b8;font-family:sans-serif;padding:20px;text-align:center;">
      <div style="font-size:1.8rem;margin-bottom:10px;">🚗 3D</div>
      <div style="font-weight:600;color:#e2e8f0;margin-bottom:6px;">Three.js Viewport (Member D)</div>
      <div style="font-size:0.75rem;color:#64748b;max-width:240px;line-height:1.4;">
        Module stub mounted. Member D will render 3D perspective camera and chase vehicle.
      </div>
    </div>
  `;
}

/**
 * Minimal compile-safe stub for Member D (3D Visualization).
 * Matches exact shared contract:
 * render3D(s: VehicleState, w: World, p: Path): void
 */
export function render3D(s: VehicleState, w: World, p: Path): void {
  // Hook for Member D's Three.js animation / render loop
}
