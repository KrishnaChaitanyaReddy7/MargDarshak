# Perception Module (`/src/perception`)

**Role**: Member C (SIH 2026 Autonomous Driving Simulation)  
**Responsibilities**: Camera-to-World Flat-Ground Homography, Replay Data Loader, Multi-Frame Velocity Estimation & Tracking, Dual-Viewport Camera/Top-Down Side Panel.

---

## 1. Module Architecture & Files

| File | Purpose |
|---|---|
| [`projection.ts`](file:///e:/SIH/src/perception/projection.ts) | Flat-ground camera homography: converts 1080p pixels to top-down world coordinates $(x, y)$ in meters with configurable camera height ($1.4\,\text{m}$), horizon line ($0.32 \cdot H$), and $f_x/f_y$ ($1000\,\text{px}$). Clips beyond $40\,\text{m}$. |
| [`replay.ts`](file:///e:/SIH/src/perception/replay.ts) | Implements `loadReplay(url)`, `getReplayWorld(i)`, `resetEgoForReplay()`, and multi-frame nearest-neighbour Euclidean obstacle velocity $(v_x, v_y)$ estimation with stable tracking IDs. |
| [`side_panel.ts`](file:///e:/SIH/src/perception/side_panel.ts) | Side panel component: displays raw front camera feed side-by-side with top-down 2D simulation view with live tuning sliders. |
| [`index.ts`](file:///e:/SIH/src/perception/index.ts) | Perception module public API exports. |
| [`test_perception.ts`](file:///e:/SIH/src/perception/test_perception.ts) | Interactive test harness loaded by `index.html`. |
| [`run_tests.js`](file:///e:/SIH/src/perception/run_tests.js) | Automated unit tests executable via Node.js. |

---

## 2. Shared Contract Alignment (`src/core/types.ts`)

Conforms strictly to Member A's shared contract:
- **World Frame**: $x = \text{right}$, $y = \text{forward}$ (up on 2D canvas), origin at ego bumper start.
- **Heading**: $\theta = 0$ facing $+y$ (forward), positive = counter-clockwise.
- **Exact signature**:
  ```typescript
  loadReplay(url: string): Promise<World[]>
  ```
- **Replay hook**:
  ```typescript
  getReplayWorld(i: number): World
  ```
- In replay mode, ego pose is reset to $(0, 0, \theta=0)$ every frame via `resetEgoForReplay()`.

---

## 3. Homography Formulas & Tunable Constants

For pixel $(u, v)$ with horizon $v_h = 0.32 \cdot H = 345.6\,\text{px}$:
$$\Delta v = v - v_h$$
$$y_{\text{world}} = \frac{H_{\text{cam}} \cdot f_y}{\Delta v} = \frac{1.4 \times 1000}{v - 345.6}$$
$$x_{\text{world}} = y_{\text{world}} \cdot \frac{u - c_x}{f_x} = \frac{1.4 \times (u - 960)}{v - 345.6}$$

- Points with $v \le v_h + 35\,\text{px}$ (i.e. $y > 40\,\text{m}$) are clipped.
- Road polygons are clipped using Sutherland-Hodgman clipping before projection to guarantee closed, bounded polygons.
- Obstacle $(x, y)$ is projected from the contact patch: bottom-centre $((x_1 + x_2)/2, y_2)$.
- Obstacle $(w, l)$ physical dimensions are assigned from class defaults (`DEFAULT_OBSTACLE_DIMENSIONS`).

---

## 4. How to Run and Test

### A. Run Automated Unit Tests
```bash
node src/perception/run_tests.js
```
Validates:
1. Camera projection math at horizon, bumper contact ($1.91\,\text{m}$), and $40\,\text{m}$ clipping boundary.
2. Frame 15 real perception validation (road polygon $\le 60$ points, car at $x=-3.71\,\text{m}, y=7.85\,\text{m}$).
3. Obstacle tracking and velocity estimation.

### B. Run Interactive Browser Simulation & Side Panel
```bash
npm run dev
```
Open the browser at `http://localhost:5173`:
- Click **"Frame 15 (Judge Test)"** to verify Frame 15 renders a plausible top-down road with obstacles.
- Click **"▶ Play"** to watch the continuous real camera feed alongside the projected 2D top-down simulation.
- Expand **"⚙ Camera Homography Calibration & Constants"** to interactively tune camera height ($1.0..2.5\,\text{m}$), horizon ratio ($0.2..0.45$), and focal length in real time.
