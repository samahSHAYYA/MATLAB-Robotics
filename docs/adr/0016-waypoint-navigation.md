# ADR-0016: Waypoint Navigation

**Status:** Completed
**Phase:** 16

## Context

Phase 15 introduced path recording/replay (recording the robot's actual trajectory and replaying it). The natural next step is to allow users to define custom waypoints by clicking in the 3D scene and have the robot autonomously navigate through them.

## Decision

1. **Click-to-place waypoints:** In `"place"` mode, clicking on the axes places a waypoint at the intersection point. Ground robots (DifferentialDrive) snap to z=0; aerial robots place at the clicked 3D position.

2. **Three-mode cycle (`'n'` key):**
   - `"off"` → `"place"` → `"navigate"` → `"off"`
   - In `"place"` mode, each click adds a numbered orange marker.
   - In `"navigate"` mode, the robot autonomously drives toward each waypoint in sequence.
   - Minimum 2 waypoints required to start navigation.

3. **Navigation logic:** The controller computes yaw error and distance to the current target waypoint. It yaws toward the target first, then drives forward. Aerial robots also adjust altitude. Each waypoint is "reached" when within `WaypointRadius` (0.15 m).

4. **Visual markers:** Orange circular markers with numbered labels, connected by a dashed orange line. Reached waypoints turn green; the active target turns red and enlarges.

5. **No new files — purely Controller + Visualizer modifications** to keep the implementation lightweight.

## Consequences

- Users can define arbitrary 2D/3D paths by clicking, enabling repeatable navigation tests.
- The `'n'` key does not conflict with `'w'` (UP for aerial robots).
- Waypoints are cleared on exiting navigation mode.
- HUD shows waypoint mode and progress (`WP: place/navigate [current/total]`).
