# Member A: Simulation, Vehicle Kinematics & Controller

This module implements the core vehicle dynamics, collision engine, and trajectory tracking controller for the SIH 2026 Unmarked Indian Road Autonomous Driving Simulation.

## Files Owned by Member A

- `src/sim/vehicle.ts`: Kinematic bicycle model with semi-implicit Euler integration.
  - Wheelbase: $L = 2.7\text{ m}$.
  - State: $(x, y, \theta, v, \delta)$.
  - Constraints: Steering rate limited to $\pm 0.8\text{ rad/s}$, steering clamped to $\pm 0.6\text{ rad}$, acceleration clamped to $[-6.0, 3.0]\text{ m/s}^2$, no reverse ($v \ge 0$).
- `src/sim/controller.ts`: Pure pursuit path tracking with speed-dependent lookahead ($L = 3 + 0.4 \times v$), path curvature slowdown, and PID speed control.
  - Emergency brake triggered when obstacles enter the $\le 3\text{ m}$ forward safety corridor.
  - Automatic slowdown when obstacles enter the $\le 8\text{ m}$ lookahead horizon.
- `src/sim/collision.ts`: 2D Oriented Bounding Box (OBB) collision detector using the Separating Axis Theorem (SAT).
- `src/sim/telemetry.ts`: Frame-by-frame telemetry telemetry aggregator exported for Member D's HUD.
- `src/sim/sim.test.ts`: Standalone verification test suite.

## Coordinate Frame Conventions

- **Units**: Meters ($\text{m}$) and Radians ($\text{rad}$). Timestep $\Delta t = 1/60\text{ s}$.
- **World Frame**:
  - $+x$: Right.
  - $+y$: Forward (Up on 2D Canvas).
  - Heading $\theta = 0$: Facing $+y$ (forward).
  - Positive angle: Counter-clockwise (CCW).
  - Steering angle $\delta > 0$: Steers vehicle to the left (CCW).

## Running and Testing

### 1. Run the Development Server
```bash
npm run dev
```
Open `http://localhost:5173` in your browser.

### 2. Controls
- **M**: Toggle between **Autopilot** and **Manual** mode.
- **W / Up Arrow**: Throttle acceleration ($+2.5\text{ m/s}^2$).
- **S / Down Arrow**: Brake deceleration ($-5.5\text{ m/s}^2$).
- **A / Left Arrow**: Smooth steer left ($+0.6\text{ rad}$).
- **D / Right Arrow**: Smooth steer right ($-0.6\text{ rad}$).
- **O**: Spawn dynamic obstacle (holy cow) ahead to test immediate avoidance replan.
- **Space**: Pause / Resume simulation.
- **R**: Reset vehicle to start.
- **1, 2, 3**: Switch between scenarios (Dense Mixed Traffic, Narrow Bottleneck, Dynamic Weaving).

### 3. Run Unit Tests
```bash
npm run test:sim
```
Verifies:
1. Circular driving radius matches theoretical $\frac{L}{\tan\delta}$ within $0.08\text{ m}$.
2. Pure pursuit tracks hardcoded S-curve with cross-track error $< 0.3\text{ m}$ (meets hackathon requirement).
