# ADR-0013: Robot Fleet App — Multi-Client Dashboard & Visual Overhaul

**Status:** accepted

**Date:** 2026-06-16

## Context

The project has 4 robot types (DifferentialDrive, Quadcopter, Quadruped, Humanoid) each demoed individually via `startRobot()`. To showcase capability, we need:

1. **Fleet control app** — spawn multiple robots simultaneously, control them from a single dashboard
2. **Extreme parallelism** — physics computations distributed across parallel pool workers
3. **Legend panel** — per-robot show/hide toggle + separate bounding-box wireframe toggle for collision visualization
4. **Visual overhaul** — Humanoid gets arms/head/joint spheres; all robots get lighting, meaningful colors, ground grid

## Decision

### 1. `RobotFleetApp.m` — Programmatic UI (not App Designer)

A single `uifigure`-based class managing the entire fleet:

| Zone | Contents |
|---|---|
| **Toolbar** | Robot spawn/remove dropdowns, Mode toggle (individual/synchronize), Formation presets |
| **Viewport grid** | 2×2 `uiaxes` layout, each showing one robot with independent camera |
| **Legend panel** | Checkbox list: per-robot visibility toggle + per-robot bounding-box toggle |
| **Control panel** | Directional pad (8 buttons), Stop, Reset, Toggle Gait |
| **Telemetry** | Live readout of selected robot: position, velocity, altitude, contact state |
| **Status bar** | FPS, simulation time, robot count, parallel pool status |

### 2. Extreme Parallelism

| Technique | Detail |
|---|---|
| **`parpool('Threads')`** | Thread-based workers (lower overhead than process) — auto-started on app launch |
| **`parfeval` per robot** | Each physics `step()` submitted as independent future |
| **`fetchNext` + batch `drawnow`** | Collect results as they complete, single render flush per frame |
| **Fallback** | Sequential execution if pool unavailable (`exist('gcp')` guard) — no crash |

Serialization approach: each robot's `State`, `Control`, and parameter struct are extracted on the client, sent to the worker as a plain struct, `computeDynamics` runs there, and the resulting `dstate` is returned for client-side integration. This avoids handle-serialization issues.

### 3. Legend Panel

Two independent toggles per robot:

- **Eye icon** (👁) — show/hide the robot's 3D graphics (wireframe + body)
- **Bounding box** (⬜) — show/hide an oriented bounding box (OBB) wireframe around the robot

Bounding boxes are computed from the robot's geometry extents rotated by the current orientation, rendered as an 8-vertex, 12-edge wireframe cube in a distinct color per robot.

### 4. Meaningful Color Scheme

| Robot | Body Color | BBox Color | Rationale |
|---|---|---|---|
| DifferentialDrive | `#E67E22` (orange) | `#E74C3C` (red) | Warm/ground — wheeled vehicle |
| Quadcopter | `#3498DB` (blue) | `#2980B9` (dark blue) | Cool/aerial — sky association |
| Quadruped | `#2ECC71` (green) | `#27AE60` (dark green) | Natural/terrestrial — animal association |
| Humanoid | `#9B59B6` (purple) | `#8E44AD` (dark purple) | Distinct/human — stands out from others |

Secondary segments (legs, arms) share the same hue at varying luminance for visual hierarchy.

### 5. Visual Overhaul

**Humanoid** (major rewrite of `buildGeometry`):
- Torso: 3D box (wider at shoulders, narrower at waist) with colored faces
- Head: sphere at neck top
- Arms: upper arm + forearm segments with spherical joints (shoulder, elbow)
- Joint markers: small spheres at each articulation
- Color: purple torso, lighter purple arms, green legs

**Quadruped** (thickened geometry):
- Body: chamfered box with colored faces
- Thighs/shin segments: thicker cylinders (was thin lines)
- Joint spheres at shoulders/knees/feet

**Quadcopter**:
- Arm booms: colored beams (not lines)
- Propeller discs: semi-transparent circles
- Center body: colored box

**DifferentialDrive**:
- Chassis: 3D box with colored faces
- Wheels: discs with rim lines
- Axle: thin cylinder

**Scene-wide**:
- `light` objects for ambient + directional illumination
- Ground grid (tiled lines on z=0 plane)
- Axis labels and tick marks
- `camlight` for depth

## Consequences

- `RobotFleetApp.m` adds ~600-800 lines of new code
- `buildGeometry` in all 4 robot classes needs updating (~50-100 lines each)
- `Collision.m` gains a `buildOBB(robot)` static method returning 8×3 vertices + 12×2 edge indices
- `startRobot.m` gets a new `'Fleet'` option
- `tasks.json` gets Phase 13
- Parallel pool requires Parallel Computing Toolbox — app degrades gracefully without it
- Bound boxes add visual overhead — toggling them off restores performance
