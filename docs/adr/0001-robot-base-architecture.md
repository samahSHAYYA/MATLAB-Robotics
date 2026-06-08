# ADR-0001: Robot Base Architecture

**Status:** accepted

**Date:** 2026-06-08

## Context

Phase 01 requires the foundational classes that every robot type will build upon. Four classes are needed before any concrete robot (Quadruped, Quadcopter, DifferentialDrive) can be implemented:

- **Direction** — enumeration for keyboard / programmatic input
- **Utils** — reusable math primitives (rotation matrices, quaternion ops, skew, RPY)
- **Robot** — abstract handle base with unified 13-element state, dynamics contract, and wireframe plotting
- **Visualizer** — scene manager for rendering robots via `hgtransform`

These must follow the package convention (`+robot/`), use MATLAB R2025 features (built-in `quaternion`, `arguments` blocks), and align with the state model and body-frame conventions established in ADR-0000 and `docs/guide/architecture.md`.

## Decision

### 1. `+robot/Direction.m` — Enumeration

MATLAB enumeration class that works with `switch`. No methods or properties beyond the values themselves.

```matlab
classdef Direction
    enumeration
        FORWARD, BACKWARD, LEFT, RIGHT
        UP, DOWN
        YAW_LEFT, YAW_RIGHT
        ROLL_LEFT, ROLL_RIGHT
        PITCH_UP, PITCH_DOWN
        STOP, RESET
    end
end
```

**Design notes:**
- 14 values total — 12 motion directions + STOP + RESET.
- No `Sealed` needed because enumerations are implicitly final in MATLAB.
- Each value maps to a unit vector or axis in the consumer (`Controller` maps these to control vectors).

### 2. `+robot/Utils.m` — Static Math Utilities

```matlab
classdef Utils < handle
    methods (Static)
        function R = rotmx(axis, angle)
            % rotmx  3x3 rotation matrix about x=1, y=2, or z=3 axis.
            %   R = Utils.rotmx(1, theta)  — rotate about x by theta rad
            %   R = Utils.rotmx(2, theta)  — rotate about y
            %   R = Utils.rotmx(3, theta)  — rotate about z
        end

        function qOut = quatMultiply(q1, q2)
            % quatMultiply  Multiply two quaternions (each [w;x;y;z]).
        end

        function R = quatToRotmx(q)
            % quatToRotmx  Convert quaternion [w;x;y;z] to 3x3 rotation matrix.
        end

        function [roll, pitch, yaw] = rotmxToRPY(R)
            % rotmxToRPY  Extract roll, pitch, yaw (rad) from 3x3 rotation matrix.
        end

        function S = skew(v)
            % skew  3x3 skew-symmetric cross-product matrix of a 3-vector.
            %   S = skew(v)  such that S * u = cross(v, u).
        end

        function S = crossEquivalent(v)
            % crossEquivalent  Alias for skew(v). Returns skew-symmetric matrix.
        end
    end
end
```

**Design notes:**
- `Sealed` so no subclassing.
- All methods `(Static)` — no instance needed.
- Input angles in radians.
- `crossEquivalent` is a semantic alias for `skew`; both are provided for readability in different contexts.
- Quaternion argument ordering is `[w; x; y; z]` (column vector) to match the state vector layout.

### 3. `+robot/Robot.m` — Abstract Handle Base

```matlab
classdef Robot < handle
    properties
        Pose          (1,1) struct    % .position [3x1], .orientation [3x1] (RPY for display)
        Params        (1,1) struct    % .geometry, .dynamics (mass, inertia, ...)
        State         (13,1) double   % [x;y;z; qw;qx;qy;qz; vx;vy;vz; wx;wy;wz]
        Control       (:,1) double    % robot-specific control vector
        InitialState  (13,1) double   % saved on construction, restored by reset()
        GraphicsTransform               % hgtransform handle (set by plot())
    end

    methods (Abstract)
        move(obj, direction, amount)
            % move  Apply a directional control.
            %   direction : robot.Direction enum value
            %   amount    : scalar magnitude (0–1 normalized)

        [verts, faces, edges] = buildGeometry(obj)
            % buildGeometry  Return wireframe geometry struct.
            %   verts : Nx3 vertices
            %   faces : Mx4 quad face indices (NaN-padded for triangles)
            %   edges : Px2 edge index pairs

        dstate = computeDynamics(obj, t, state, control)
            % computeDynamics  ODE right-hand side.
            %   t      : current time (s)
            %   state  : 13x1 state vector
            %   control: robot-specific control vector
            %   dstate : 13x1 time derivative of state
    end

    methods
        function obj = Robot(params)
            % Robot  Constructor. Store params, build geometry.
            %   params : struct with .geometry and .dynamics fields
        end

        function step(obj, t, dt)
            % step  Integrate dynamics one timestep via RK4.
            %   Calls DynamicsEngine.rk4Step internally.
        end

        function reset(obj)
            % reset  Restore InitialState and clear Control.
        end

        function hg = plot(obj, ax)
            % plot  Create hgtransform + wireframe patch objects in axes.
            %   Returns the hgtransform handle.
        end

        function s = getState(obj)
            % getState  Return current 13x1 state vector.
        end

        function setState(obj, s)
            % setState  Set state from a 13x1 vector. Updates Pose.orientation.
        end
    end
end
```

**State vector layout** (13 elements):

| Index | Field | Frame |
|---|---|---|
| 1–3 | `[x; y; z]` position | world |
| 4–7 | `[qw; qx; qy; qz]` orientation | world→body |
| 8–10 | `[vx; vy; vz]` linear velocity | body |
| 11–13 | `[wx; wy; wz]` angular velocity | body |

**Design notes:**
- Inherits `handle` — all state mutable by reference.
- `Pose.orientation` stored as RPY `[roll; pitch; yaw]` for display/debug; the canonical orientation is in `State(4:7)` as a quaternion.
- `step()` delegates to `DynamicsEngine.rk4Step` (static) to avoid coupling Robot to the engine object.
- `GraphicsTransform` is untyped (`hgtransform` handle) — set by `plot()`, read by `Visualizer.update()`.

### 4. `+robot/Visualizer.m` — Wireframe Renderer

```matlab
classdef Visualizer < handle
    properties
        AxesHandle      (1,1) matlab.graphics.axis.Axes
        TransformGroup  (1,1) matlab.graphics.primitive.Transform    % hgtransform container
        Robots                 cell                                % cell array of robot.Robot handles
        GroundHandle   (1,1) matlab.graphics.primitive.Patch
    end

    methods
        function obj = Visualizer(ax)
            % Visualizer  Constructor. Set up 3D axes, grid, ground plane.
        end

        function addRobot(obj, robot)
            % addRobot  Add a robot to the scene.
            %   Calls robot.plot(obj.TransformGroup) and stores reference.
        end

        function update(obj, robot)
            % update  Refresh wireframe transform for one robot.
            %   Reads robot.State(1:7), builds 4x4 Matrix, sets hgtransform.Matrix.
        end

        function clear(obj)
            % clear  Remove all robot graphics from the scene.
        end
    end
end
```

**Implementation strategy:**
- Constructor creates an `hgtransform` as `TransformGroup` to act as parent for all robot wireframes.
- `addRobot` calls `robot.plot(obj.TransformGroup)` so each robot's `patch` children are parented under the group.
- `update` extracts position and quaternion from `robot.State`, builds:
  ```matlab
  T = eye(4);
  T(1:3, 1:3) = rotmat(q, 'point');
  T(1:3, 4) = position;
  robot.GraphicsTransform.Matrix = T;
  ```
- `clear` deletes children of `TransformGroup` and empties `Robots` cell array.

## Consequences

- All concrete robots inherit a uniform state interface, enabling generic `DynamicsEngine` and `Visualizer`.
- Adding a new robot type requires implementing only `move`, `buildGeometry`, and `computeDynamics`.
- The `Direction` enum provides a single vocabulary for keyboard input, controller programs, and tests.
- `Utils` static methods are independently testable and reusable outside the class hierarchy.
- No breaking changes to existing ADR-0000 decisions — this ADR implements the architecture described in `docs/guide/architecture.md` and `docs/guide/api.md`.
- The `+robot/` package directory must be created with these four files before any consuming code can import from the package.

## Files created

- `+robot/Direction.m`
- `+robot/Utils.m`
- `+robot/Robot.m`
- `+robot/Visualizer.m`
