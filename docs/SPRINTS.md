> **Status: Historical reference.** The sprint plan below was an early
> roadmap. All implementation work has been tracked via
> [tasks.json](../.agent/tasks.json) and
> [CHANGELOG.md](../CHANGELOG.md).

# matlab-robotics — Sprint Tracking

## Sprint 1 — Multi-Robot Core

| # | Task | Status | Notes |
|---|---|---|---|
| 1.1 | `MultiController` class (multi-robot render loop) | pending | |
| 1.2 | `startRobot.m` accepts robot array | pending | |
| 1.3 | Active robot switching (Tab, 1-9 keys) | pending | |
| 1.4 | Visual indicator for active robot | pending | |
| 1.5 | Keyboard routes to active robot only | pending | |

## Sprint 2 — Command Sequencing

| # | Task | Status | Notes |
|---|---|---|---|
| 2.1 | `ScheduledCommand` data class | pending | |
| 2.2 | `CommandSequencer` (insert, dispatch, reset) | pending | |
| 2.3 | Record mode (capture keystrokes with timestamps) | pending | |
| 2.4 | Save / load sequence .mat | pending | |
| 2.5 | Playback mode | pending | |

## Sprint 3 — Coordinate Frames

| # | Task | Status | Notes |
|---|---|---|---|
| 3.1 | `FrameAxes` class — world frame (RGB arrows + labels) | pending | |
| 3.2 | Body frame per robot (child of hgtransform) | pending | |
| 3.3 | Toggle visibility (x / z keys) | pending | |

## Sprint 4 — Trajectory Visualization

| # | Task | Status | Notes |
|---|---|---|---|
| 4.1 | `TrailRecorder` class (append, line handle, fading) | pending | |
| 4.2 | Per-robot trail colours (auto palette) | pending | |
| 4.3 | Toggle / clear trails (t / T keys) | pending | |

## Sprint 5 — Simulation Controls & Polish

| # | Task | Status | Notes |
|---|---|---|---|
| 5.1 | Pause / resume | pending | |
| 5.2 | Speed control ([ / ] keys, 0.1×–10×) | pending | |
| 5.3 | Status display (time, active robot, mode, speed) | pending | |
| 5.4 | Export video (MATLAB `VideoWriter`) | pending | |

## Sprint 6 — UI & Sensors (stretch)

| # | Task | Status | Notes |
|---|---|---|---|
| 6.1 | In-figure UI panel | pending | |
| 6.2 | Runtime parameter editing | pending | |
| 6.3 | Collision detection between robots | pending | |
| 6.4 | Sensor visualization (cones / rays / FOV) | pending | |
