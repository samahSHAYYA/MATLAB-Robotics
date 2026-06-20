# ADR-0015: Visual Refinements and Controller Innovations

**Status:** accepted

**Date:** 2026-06-20

## Context

Phase 14 completed UI responsiveness improvements, async pool startup, custom spawn dialog, git hooks, and architecture documentation. The core physics and keyboard control loop is solid across all 4 robot types.

However, several visual and interaction gaps remain:

1. **No position history** — users cannot see where a robot has travelled (useful for debugging gait trajectories and controller response)
2. **No spatial context** — robots float above a featureless ground plane; no drop shadow gives depth perception
3. **No status feedback** — running lights would visually indicate whether a robot is moving, stopped, or in gait mode
4. **No telemetry HUD** — in single-robot mode (`startRobot`), the user has no numeric readout of speed, altitude, or state
5. **Fixed camera** — no way to orbit, chase, or view from above without manual `view()` calls
6. **No automated path tools** — path recording and replay would help demonstrate gait repeatability and save/load motion

These features were deferred from Phase 13 (visual overhaul + fleet app) to keep that phase focused.

## Decision

### 1. Trail Buffer (position history line)

- **Robot base class** gains a `TrailBuffer` (`:,3` double, max 300 points) and a `TrailHandle` (line handle parented to axes).
- **`Robot.step()`** appends `State(1:3)` to `TrailBuffer` when `norm(State(8:10)) > 0.02`, overflowing via circular indexing.
- **`Visualizer.update()`** calls a new `updateVisuals(robot, axes)` method that refreshes the trail line's X/Y/ZData from the buffer.

### 2. Ground Shadow

- **Robot base class** gains a `ShadowHandle` (patch handle parented to axes).
- Each robot's **`plot()`** creates a type-appropriate shadow shape: ellipse for ground robots (matching body extents), circle for Quadcopter.
- Shadows are semi-transparent (`FaceAlpha=0.15`, `FaceColor=[0.3 0.3 0.3]`) and sit on `z=0`.
- `updateVisuals()` recenters the shadow to `State(1:2)` each frame.

### 3. Running Lights

- Each robot's **`buildGeometry()`** (or `plot()`) creates 2-4 small colored patches at front/rear or arm tips.
- `Robot` base class gains `LightsOn` (`logical`, toggled by `'l'` key) and `RunningLightHandles` cell array.
- `updateVisuals()` sets patch `FaceColor` based on movement/gait state: green = moving, amber = stopped, red = gait active.

### 4. HUD Overlay (single-robot Controller)

- `Controller` gains `HudActive` (`logical`) and `HudHandles` struct with 4 text annotations on the figure (top-right `Units = 'normalized'`).
- Annotations show: speed (`m/s`), altitude (`m`), battery placeholder (`%`), mode string.
- `'h'` key toggles HUD visibility.

### 5. Camera Modes

- `Visualizer` gains `CameraMode` string property (`"free"`, `"chase"`, `"orbit"`, `"top"`). Default: `"free"`.
- `Controller` gains a `CameraModeIdx` counter; `'c'` key cycles modes.
- In **chase** mode: camera places 1.5 m behind, 0.5 m above robot body frame.
- In **orbit** mode: camera circles at 2 m radius, height 0.8 m, angular rate `0.3 rad/s`.
- In **top** mode: `view(0, 90)`, `campos` above robot.
- Camera updates happen post-render in the controller loop.

### 6. Path Recording/Replay (single-robot Controller)

- `Controller` gains `PathMode` (`"manual"`, `"record"`, `"replay"`), `RecordedPath` (N×4 buffer), `ReplayIdx`, `ReplayTime`.
- `'p'` key cycles: manual → record → replay → manual.
- During **record**: append `[t, x, y, z]` every physics step when moving.
- During **replay**: interpolate target pose from recorded path, set robot state directly (bypassing physics for precise replay).

### 7. Waypoint Navigation (stretch, FleetApp only)

- FleetApp `SceneAxes` gains `ButtonDownFcn` for click-to-set-waypoint on ground plane.
- Waypoint list stored per robot; a proportional heading controller steers the robot toward the next waypoint.
- `'m'` key toggles waypoint-follow mode for the selected robot.

## Consequences

### Easier
- Users can visually verify robot trajectories and gait cycles
- Depth perception improves with drop shadows
- Status-at-a-glance via running lights
- Single-robot demos gain real-time telemetry
- Path replay makes demo repeatability trivial
- Camera modes improve presentation quality

### Harder
- Every robot subclass must implement `updateVisuals()` — but it's a single method with 3-4 lines of handle updates
- `Visualizer.update()` does slightly more work per frame (trail + shadow update) — negligible for 4 robots
- `Controller.run()` loop grows conditional branches for HUD, camera, and path mode — but each is gated by a simple flag check
- Running lights add ~4 patch handles per robot (16 total for fleet) — no performance concern

### Must Change
- `+robot/Robot.m` — add 5 properties, update `step()`, add `updateVisuals()` abstract/virtual method
- `+robot/Visualizer.m` — add `CameraMode`, update `update()` to call `updateVisuals()`
- `+robot/Controller.m` — add HUD, camera mode cycling, path recording/replay
- All 4 robot subclass `plot()` methods — add shadow, trail, running lights, `updateVisuals()` override
- `RobotFleetApp.m` — (stretch) waypoint click handler

### Migration
- Existing code using `Visualizer.update()` will continue to work (calls `isprop(robot, 'updateVisuals')` guard)
- Existing subclasses without `updateVisuals()` will silently skip trail/shadow updates
