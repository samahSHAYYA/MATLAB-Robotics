# ADR-0002: DifferentialDrive + DynamicsEngine

**Status:** accepted

**Date:** 2026-06-08

## Context

Phase 01 built the abstract `Robot` base class with a 13-element state, the `Direction` enum, `Utils` math library, and `Visualizer`. Phase 02 must deliver the first concrete robot (`DifferentialDrive` — a 3-DOF planar wheeled robot) and a reusable numerical integrator (`DynamicsEngine`) that all robots will use for stepping physics.

Two forces drive the design:

- **Reusability:** the RK4 integrator is identical across all robot types. It must live outside the hierarchy as a static utility, not duplicated in each subclass.
- **Planar constraint:** DifferentialDrive operates on the ground (z=0) with only 3 DOF (x, y, yaw). For now it inherits directly from `Robot`; a `GroundRobot` intermediate abstract class will be introduced in Phase 05 when Quadruped arrives.

## Decision

### 1. `+robot/DynamicsEngine.m` — Static RK4 Integrator

A `Sealed` class with only `Static` methods. No constructors, no instance state — used as `DynamicsEngine.rk4Step(...)` and `DynamicsEngine.integrate(...)`.

```matlab
classdef (Sealed) DynamicsEngine
    methods (Static)
        function stateOut = rk4Step(dynFun, t, state, control, dt)
            % rk4Step  Single fourth-order Runge-Kutta step.
            %   dynFun   : function handle @(t, state, control) -> dstate (13x1)
            %   t        : current simulation time (s)
            %   state    : 13x1 state vector
            %   control  : robot-specific control vector (:,1)
            %   dt       : time step size (s)
            %   stateOut : 13x1 integrated state at t+dt
        end

        function states = integrate(dynFun, tSpan, state0, control, dt)
            % integrate  Integrate over a time span, returning trajectory.
            %   dynFun : function handle @(t, state, control) -> dstate
            %   tSpan  : [t0, tf] start and end times
            %   state0 : 13x1 initial state
            %   control: control vector applied over entire span
            %   dt     : time step size (s)
            %   states : Nx13 matrix, each row is state at time t0 + k*dt
        end
    end
end
```

**Standard RK4 formula:**

```
k1 = f(t, s, u)
k2 = f(t + dt/2, s + (dt/2)*k1, u)
k3 = f(t + dt/2, s + (dt/2)*k2, u)
k4 = f(t + dt, s + dt*k3, u)
s_next = s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
```

**`integrate` implementation strategy:**
- Allocate `states = zeros(ceil((tf-t0)/dt) + 1, 13)`, set `states(1,:) = state0'`.
- Loop k = 2:N, calling `rk4Step` each iteration, appending to the row.
- Return the full trajectory matrix. This is useful for plotting, analysis, or batch simulation.

**Design notes:**
- `dynFun` signature is `@(t, state, control)` — this matches `Robot.computeDynamics` when adapted: `@(t, s, u) obj.computeDynamics(t, s, u)`.
- No validation of input dimensions inside the hot loop; validation is the caller's responsibility.
- `Sealed` to prevent subclassing — no meaningful extension point.

### 2. `+robot/DifferentialDrive.m` — Concrete 3-DOF Planar Robot

Inherits from `Robot` (abstract). Represents a two-wheel differentially-steered robot moving in the x-y plane at z=0.

#### Params struct

```matlab
params.geometry.wheelRadius   % wheel radius (m)
params.geometry.trackWidth    % distance between left and right wheels (m)
params.dynamics.mass          % robot mass (kg)
params.dynamics.inertia       % moment of inertia about body z-axis (kg·m²)
params.dynamics.maxTorque     % maximum wheel torque (N·m)
```

#### State

The standard 13-element state vector. For a planar robot on z=0, the active degrees of freedom are:

| Index | Field | Value |
|---|---|---|
| 1 | x | world-frame x (m) |
| 2 | y | world-frame y (m) |
| 3 | z | always 0 |
| 4–7 | qw, qx, qy, qz | quaternion; only yaw is free, roll = pitch = 0 |
| 8 | vx | body-frame forward velocity (m/s) |
| 9 | vy | body-frame lateral velocity — 0 (no sideslip) |
| 10 | vz | always 0 |
| 11 | wx | always 0 |
| 12 | wy | always 0 |
| 13 | wz | angular velocity about body z (rad/s) |

The quaternion is computed as `[cos(yaw/2); 0; 0; sin(yaw/2)]` — pure z-rotation.

#### Control

2-element column vector: `[tau_L; tau_R]` where:
- `tau_L` = left wheel torque (N·m)
- `tau_R` = right wheel torque (N·m)

#### Method: `move(direction, amount)`

Maps a `Direction` enum to control torques. `amount` is a scalar in [0, 1], multiplied by `maxTorque`.

| Direction | tau_L | tau_R |
|---|---|---|
| FORWARD | +amount * maxTorque | +amount * maxTorque |
| BACKWARD | -amount * maxTorque | -amount * maxTorque |
| LEFT | -amount * maxTorque | +amount * maxTorque |
| YAW_LEFT | -amount * maxTorque | +amount * maxTorque |
| RIGHT | +amount * maxTorque | -amount * maxTorque |
| YAW_RIGHT | +amount * maxTorque | -amount * maxTorque |
| STOP | 0 | 0 |
| UP, DOWN, ROLL_*, PITCH_* (aerial) | no-op — no torque applied | no-op — no torque applied |
| RESET | resets state, clears control | resets state, clears control |

#### Method: `buildGeometry()`

Returns `[vertices, faces, edges]` for a simple rectangular body with two wheel rectangles.

**Body:** 0.4 m × 0.3 m × 0.1 m, centered at origin.
- 8 corners of a box, aligned with body axes.
- Faces: 6 quads (NaN-padded to M×4), one per box face.
- Edges: 12 edges of the box.

**Wheels (left and right):**
- Left wheel: thin rectangle extending from body left side. Offset `(-trackWidth/2, 0, 0)`, dimensions 0.05 m × 0.2 m × 0.2 m (width × diameter × thickness).
- Right wheel: mirror of left.
- Vertices appended to the vertex list.
- Faces and edges appended to their respective arrays.

Output convention (matches `Robot` abstract):
- `vertices`: N×3 double — all vertices concatenated
- `faces`: M×4 double — quad indices (1-based), NaN-padded for any face with < 4 vertices
- `edges`: P×2 double — pairs of vertex indices for wireframe lines

#### Method: `computeDynamics(t, state, control)`

Unicycle-like planar dynamics in the body frame.

```
% Extract state
vx = state(8);                  % body forward velocity
wz = state(13);                 % body angular velocity about z
q = state(4:7);                 % orientation quaternion

% Rotation matrix from body to world
R_body_to_world = quatToRotmx(q);

% Tractive force from wheel torques
F_drive = (control(1) + control(2)) / wheelRadius;

% Yaw torque from differential wheel torques
T_yaw = (control(2) - control(1)) * trackWidth / (2 * wheelRadius);

% Body-frame forces
F_body = [F_drive; 0; 0];       % forward tractive force
T_body = [0; 0; T_yaw];         % yaw torque only

% Linear acceleration in body frame (no sideslip, no z acceleration)
dvel_x = F_body(1) / mass - wy * vz + wz * vy;  % = F_drive/mass (since vy=0, vz=0)
dvel_y = 0;                                       % no lateral slip
dvel_z = 0;                                       % ground reaction cancels gravity

% Angular acceleration
domega_x = 0;
domega_y = 0;
domega_z = T_body(3) / inertia;

% Position derivative (velocity rotated to world frame)
dpos = R_body_to_world * [vx; 0; 0];

% Quaternion derivative
omega_Q = [0; 0; 0; wz];
dq = 0.5 * quatMultiply(q, omega_Q);

% Assemble derivative
dstate = [dpos; dq; dvel_x; dvel_y; dvel_z; domega_x; domega_y; domega_z];
```

**Simplifications for the planar case:**
- vy is held at 0 (no sideslip model — instant ground friction kills lateral velocity).
- vz = 0 (ground plane constraint).
- wx = wy = 0 (no roll/pitch dynamics).
- Gravity is not explicitly computed because planar constraint counters it — `dvel_z = 0` models a rigid ground plane.
- Centrifugal/coriolis terms (`cross(omega, vel)`) are zero because `omega = [0;0;wz]` and `vel = [vx;0;0]` have their cross product aligned perpendicular to the plane; the body-frame cross product `omega × vel` yields `[-vy*wz; vx*wz; 0]` which reduces to `[0; vx*wz; 0]`. For now we ignore the centrifugal term `vx*wz` in dvel_y because we enforce vy = 0 directly; a future GroundRobot refinement may add lateral tire friction.

#### Override: `step(t, dt)`

Overrides `Robot.step` to actually integrate state:

```matlab
function step(obj, t, dt)
    dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
    obj.State = DynamicsEngine.rk4Step(dynFun, t, obj.State, obj.Control, dt);
    obj.updatePoseFromState();
end
```

The base class `Robot.step` is a no-op (it only calls `updatePoseFromState`). DifferentialDrive replaces it with real physics.

#### Override: `plot(ax)`

Override `Robot.plot` to create meshed patch objects from `buildGeometry` output, parented under the `hgtransform`:

```matlab
function hg = plot(obj, ax)
    hg = plot@robot.Robot(obj, ax);         % create hgtransform via base
    [verts, faces, edges] = obj.buildGeometry();
    % Create patch for faces
    patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
          'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    % Create line objects for edges
    for i = 1:size(edges, 1)
        line('Parent', hg, ...
             'XData', verts(edges(i,:), 1), ...
             'YData', verts(edges(i,:), 2), ...
             'ZData', verts(edges(i,:), 3), ...
             'Color', 'k', 'LineWidth', 1.5);
    end
end
```

**Design note:** Each edge drawn as a separate `line` primitive. For models with many edges, consider `line` with `NaN`-separated coordinate arrays. For this simple geometry the loop is acceptable.

### 3. File structure

```
+robot/
  Direction.m           (Phase 01)
  Utils.m               (Phase 01)
  Robot.m               (Phase 01)
  Visualizer.m          (Phase 01)
  DynamicsEngine.m      (Phase 02 — NEW)
  DifferentialDrive.m   (Phase 02 — NEW)
```

No changes to existing Phase 01 files. The new files are additive.

### 4. Integration test pattern

A simple validation script `test_differential_drive.m` (placed under `.agent/skills/robot-dynamics/scripts/`) should verify:

1. **Geometry output:** `buildGeometry` returns `Nx3` verts, `Mx4` faces, `Px2` edges with valid index ranges.
2. **Move commands:** each `move(direction, 1.0)` sets the correct control torque pair; `STOP` zeros it; aerial directions leave control unchanged.
3. **Forward dynamics:** a `FORWARD` command produces positive x velocity after one `step`.
4. **Yaw dynamics:** a `YAW_LEFT` command produces positive wz (counter-clockwise angular velocity).
5. **RK4 accuracy:** `integrate` with zero control preserves initial state (within numerical precision).
6. **Reset:** after `reset()`, state equals `InitialState` and Control is empty.

## Consequences

- `DynamicsEngine` provides a single, tested RK4 implementation for all future robots — no duplication.
- `DifferentialDrive` serves as the template for all future concrete robot subclasses, demonstrating the full pattern: `buildGeometry`, `move`, `computeDynamics`, `step` override, `plot` override.
- Planar approximations (no sideslip, instant ground reaction) are acceptable for Phase 02; a `GroundRobot` abstract layer in Phase 05 will formalize ground contact models.
- `step()` being overridden rather than implemented in `Robot` base is a deliberate choice — the base cannot know the correct dynamics function. The phase-05 `Robot.step` may be strengthened to call a method that returns `dynFun` or otherwise delegate.
- Wireframe rendering with individual `line` primitives is simple but may become expensive for complex models; optimization (single `line` with `NaN` separators) is deferred.
