# ADR-0017: Multi-Robot Scene Support

**Status:** accepted

**Date:** 2026-06-21

## Context

The `startRobot.m` entry point only supports a single robot. The `RobotFleetApp` demonstrates multi-robot spawning via a UI dashboard, but the keyboard-driven `startRobot` demo lacks native multi-robot support. Users who want to run multiple robots in a shared 3D scene must use the fleet app, which is UI-based and doesn't support keyboard-driven control.

The Visualizer already supports multiple robots via its `Robots` cell array and per-robot `hgtransform` instances. The Controller is the bottleneck — it holds a single `Robot` property and its main loop only steps/updates one robot.

This ADR extends the Controller and `startRobot` entry point to support multiple robots in a single scene, with keyboard-driven active robot switching.

## Decision

1. **`startRobot` accepts a cell array** of robot type strings (e.g., `startRobot({'Quadruped', 'Quadcopter'})`). Single-string calls remain fully backward compatible.

2. **Controller holds a `Robots` cell array** instead of a single `Robot`. An `ActiveIdx` property (default 1) tracks the active robot.

3. **Active robot switching** via Tab (cycle forward), Shift+Tab (cycle backward), and number keys 1-9 (jump to index).

4. **Keyboard commands route to the active robot only.** All other robots continue their dynamics but receive STOP commands.

5. **Main loop iterates all robots** for physics stepping and visual update. The render loop steps every robot each frame, not just the active one.

6. **Camera tracks the active robot.** Chase, orbit, and top camera modes follow the active robot's position and orientation.

7. **HUD displays active robot name and index** (e.g., "Quadruped [1/2]").

8. **Waypoints, path recording/replay, and gait toggle** operate on the active robot only.

9. **No changes to Visualizer API** — the Controller loops over robots calling `Visualizer.update(rbt)` per robot.

## Consequences

**Easier:**
- Users can run multiple robots in a single scene from the command line
- Backward compatible — all existing single-robot calls work unchanged
- Reuses existing multi-robot infrastructure (per-robot `hgtransform`, `Visualizer.Robots` cell array)

**Harder:**
- Camera modes can only follow one robot at a time (the active one)
- Waypoints, path recording, and HUD are per-active-robot; switching robots loses the current waypoint/path context
- The figure title cannot list all robot types concisely for many robots

**Not addressed in this phase:**
- Per-robot waypoint paths (shared path for active robot only)
- Visual bounding box / highlight for active robot (future refinement)
- Collision detection between robots (already exists in `robot.Collision`)
- Script playback across multiple robots (already exists in `RobotFleetApp`)
