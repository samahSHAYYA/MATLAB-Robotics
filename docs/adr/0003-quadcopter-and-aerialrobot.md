# ADR-0003: Quadcopter + AerialRobot

**Status:** accepted

**Date:** 2026-06-08

## Context

Phase 03 delivered the interactive `Controller` + `startRobot.m` for `DifferentialDrive`. Phase 04 must deliver the first aerial robot (`Quadcopter` — a 6-DOF quadrotor) and an intermediate abstract base (`AerialRobot`) that marks the semantic distinction between ground and aerial vehicles.

Three design forces:

- **Full 6-DOF dynamics:** Unlike DifferentialDrive (planar, 3-DOF), the quadcopter must simulate x/y/z translation and roll/pitch/yaw rotation with full rigid-body coupling.
- **Hierarchical correctness:** The class hierarchy in `docs/guide/architecture.md` shows `AerialRobot` between `Robot` and `Quadcopter`. Phase 04 is the right time to introduce it, so Phase 05's `GroundRobot` will mirror the pattern.
- **Step override pattern:** `Robot.step` is a no-op (only calls `updatePoseFromState`). `DifferentialDrive` overrides it with real RK4 integration. `AerialRobot` should provide the same override as the default for all aerial subclasses.

**Note on task sequencing:** The tasks.json Phase 05 lists both `GroundRobot` and `AerialRobot`. This ADR addresses `AerialRobot` early (in Phase 04) to serve as `Quadcopter`'s parent. Phase 05 will add `GroundRobot` as a sibling abstraction, at which point `DifferentialDrive` will be re-parented from `Robot` → `GroundRobot`.

## Decision

### 1. `+robot/AerialRobot.m` — Abstract base for flying robots

Inherits from `Robot`. No new properties. No new abstract methods — all abstract methods are inherited from `Robot`.

Provides two concrete method overrides:

**`step(t, dt)` — integrate dynamics:**
```matlab
function step(obj, t, dt)
    arguments
        obj
        t (1,1) double
        dt (1,1) double
    end
    if isempty(obj.Control)
        u = zeros(obj.getControlDim(), 1);
    else
        u = obj.Control;
    end
    dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
    s = DynamicsEngine.rk4Step(dynFun, t, obj.State, u, dt);
    obj.setState(s);
end
```

Follows the same pattern as `DifferentialDrive.step`. The base `Robot.step` is a no-op; `AerialRobot` replaces it with real physics integration. All aerial subclasses inherit this default and do not need to re-implement `step()`.

**`hover()` — zero the control vector:**
```matlab
function hover(obj)
    arguments
        obj
    end
    obj.Control = zeros(obj.getControlDim(), 1);
end
```

A convenience method for aerial vehicles. Maps to the same effect as `move(STOP, 0)`. `hover` is more semantically meaningful for flight.

**Helper — `getControlDim()`:**
```matlab
function n = getControlDim(obj)
    arguments
        obj
    end
    n = 4;  % default for aerial (Quadcopter)
end
```

Overridable by subclasses. Returns the expected control vector length so that `step()` and `hover()` can allocate a zero vector when `Control` is empty.

**Design notes:**
- `AerialRobot` is not `Abstract` in MATLAB `classdef` sense — it has no `methods (Abstract)` beyond those inherited from `Robot`. It is "semantically abstract": it should not be instantiated directly, but this is enforced by convention (no `AerialRobot(params)` usage), not by language restriction.
- The class exists to (a) provide the default `step` override, (b) sit in the hierarchy so that `GroundRobot` has a mirror, and (c) allow `isa(obj, 'robot.AerialRobot')` checks in `Controller` and `Visualizer`.

### 2. `+robot/Quadcopter.m` — Concrete 6-DOF aerial robot

Inherits from `AerialRobot`. A quadcopter with 4 rotors in standard 'X' configuration.

#### Params struct

```matlab
params.geometry.armLength        % distance from center to rotor (m)
params.geometry.bodySize         % body box dimensions [length, width, height] in m
params.dynamics.mass             % total mass (kg)
params.dynamics.inertia          % 3×3 inertia matrix (kg·m²)
params.dynamics.maxThrust        % max thrust per rotor (N)
params.dynamics.maxTorque        % max reaction torque per rotor (N·m)
```

#### Constructor

```matlab
function obj = Quadcopter(params)
    arguments
        params (1,1) struct
    end
    obj@robot.AerialRobot(params);
    obj.Control = zeros(4, 1);
end
```

Unpacks `params` into instance properties for the dynamics model. Control starts as 4-element zero vector (all rotors off).

#### Properties

Stored as instance properties unpacked from `Params` for fast access in the hot dynamics loop:

```matlab
properties
    armLength  (1,1) double
    bodySize   (1,3) double
    mass       (1,1) double
    inertia    (3,3) double
    maxThrust  (1,1) double
    maxTorque  (1,1) double
end
```

#### State

Standard 13-element vector. All 6 DOF are active — every element can vary:

| Index | Field | Notes |
|---|---|---|
| 1–3 | x, y, z | World frame, z positive up |
| 4–7 | qw, qx, qy, qz | Unit quaternion, world→body |
| 8–10 | vx, vy, vz | Body-frame linear velocity |
| 11–13 | wx, wy, wz | Body-frame angular velocity |

No constraints. The quadcopter can translate and rotate freely in 3D.

#### Control

4-element column vector: `[T1; T2; T3; T4]` where each Ti is thrust (N) from rotor i.

| Rotor | Position (body frame) | Spin direction |
|---|---|---|
| T1 | front-left: `[+armLen; -armLen; 0]` | CCW (+z torque) |
| T2 | front-right: `[+armLen; +armLen; 0]` | CW (-z torque) |
| T3 | rear-left: `[-armLen; -armLen; 0]` | CCW (+z torque) |
| T4 | rear-right: `[-armLen; +armLen; 0]` | CW (-z torque) |

Rotor positions in body frame (x=forward, y=right, z=up). Spinning direction determines the sign of the reaction torque about the body z-axis. Rotors 1 and 3 spin counter-clockwise; rotors 2 and 4 spin clockwise — this cancels net yaw torque at hover.

#### Method: `move(direction, amount)`

Maps a `Direction` enum to rotor thrusts. `amount` is clamped to [0, 1], then multiplied by `maxThrust`.

| Direction | T1 | T2 | T3 | T4 |
|---|---|---|---|---|
| UP | +amt | +amt | +amt | +amt |
| DOWN | -amt | -amt | -amt | -amt |
| FORWARD | 0 | 0 | +amt | +amt |
| BACKWARD | +amt | +amt | 0 | 0 |
| LEFT | +amt | 0 | +amt | 0 |
| RIGHT | 0 | +amt | 0 | +amt |
| YAW_LEFT (CCW) | -amt | +amt | +amt | -amt |
| YAW_RIGHT (CW) | +amt | -amt | -amt | +amt |
| ROLL_LEFT | +amt | -amt | +amt | -amt |
| ROLL_RIGHT | -amt | +amt | -amt | +amt |
| PITCH_UP | -amt | -amt | +amt | +amt |
| PITCH_DOWN | +amt | +amt | -amt | -amt |
| STOP | 0 | 0 | 0 | 0 |
| RESET | call `reset()` |  |  |  |

**Semantics:**
- Each command adds a delta to the current rotor thrusts (not absolute set). The amount is signed per the table.
- After applying the command, each rotor thrust is clamped to `[0, maxThrust]` (rotors produce positive thrust only).
- `STOP` sets all thrusts to zero (free fall).
- `RESET` calls inherited `reset()` which restores `InitialState` and clears `Control`.
- Ground-only commands (LEFT, RIGHT when used as lateral translation) are interpreted as body-frame lateral translation. The mapping produces differential thrust on the left vs. right rotor pairs.

#### Method: `buildGeometry()`

Returns `[verts, faces, edges]` for a wireframe quadcopter:

**Central body:** A box with dimensions `params.geometry.bodySize`. 8 corner vertices, 6 quad faces, 12 edges — same pattern as `DifferentialDrive.buildGeometry`.

**Arms:** 4 arms extending from the body center to rotor positions at distance `armLength` along the X-configuration diagonals. Each arm can be represented as a thin rectangular box (e.g., 0.01 m × 0.01 m × armLength). The arms are oriented along the diagonal vectors `[±armLen; ±armLen; 0]`. In practice, plotting a line or thin cylinder from the origin to each rotor position is sufficient for wireframe.

**Rotors:** 4 small disks (or squares) at the arm tips. Represent as a small set of vertices forming a circle with radius ~0.03 m. For wireframe, edges around the circumference plus a cross inside.

The exact vertex/face/edge layout:

```
% Body box
bodyVerts = 8 corners of bodySize box, centered at origin
bodyFaces = [1 2 3 4; 5 8 7 6; 1 5 6 2; 3 7 8 4; 1 4 8 5; 2 6 7 3]
bodyEdges = 12 edges of box

% For each rotor i at position ri (body frame):
%   Arm: line from [0,0,0] to ri (optionally a thin box)
%   Rotor disk: small circle fans at ri

% Concatenation:
verts = [bodyVerts; armVerts; rotorVerts]
faces = [bodyFaces; armFaces; rotorFaces]  (with offset indices)
edges = [bodyEdges; armEdges; rotorEdges]  (with offset indices)
```

Implementation tip: keep arms as simple lines (just edge pairs) and rotors as a circle of 8 vertices with radial edges. This keeps the wireframe clean and fast.

#### Method: `computeDynamics(t, state, control)`

Full 6-DOF rigid-body dynamics with rotor thrust and reaction torque:

```matlab
function dstate = computeDynamics(obj, t, state, control)
    % Extract state
    pos = state(1:3);
    q = quaternion(state(4:7)');
    vel = state(8:10);
    omega = state(11:13);

    % Body-to-world rotation matrix
    R = rotmat(q, 'point');

    % Rotor positions in body frame (X-configuration)
    L = obj.armLength;
    r1 = [ L; -L; 0];   % front-left
    r2 = [ L;  L; 0];   % front-right
    r3 = [-L; -L; 0];   % rear-left
    r4 = [-L;  L; 0];   % rear-right

    T = control;  % [T1; T2; T3; T4]

    % Total thrust in body z direction
    F_body = [0; 0; sum(T)];

    % Moment from each rotor thrust: cross(ri, [0; 0; Ti])
    M1 = cross(r1, [0; 0; T(1)]);
    M2 = cross(r2, [0; 0; T(2)]);
    M3 = cross(r3, [0; 0; T(3)]);
    M4 = cross(r4, [0; 0; T(4)]);

    % Reaction torque: rotors 1,3 spin CCW (+z); 2,4 spin CW (-z)
    k = obj.params.dynamics.kTorque;  % torque ratio, default 0.01
    tau1 = [0; 0; -k * T(1)];
    tau2 = [0; 0;  k * T(2)];
    tau3 = [0; 0;  k * T(3)];
    tau4 = [0; 0; -k * T(4)];

    T_body = M1 + M2 + M3 + M4 + tau1 + tau2 + tau3 + tau4;

    % Gravity in body frame
    g_world = [0; 0; -9.81];
    g_body = R' * g_world;

    % Linear acceleration (body frame)
    dvel = F_body / obj.mass + g_body - cross(omega, vel);

    % Angular acceleration (Euler equation)
    I = obj.inertia;
    domega = I \ (T_body - cross(omega, I * omega));

    % Position derivative (world frame)
    dpos = R * vel;

    % Quaternion derivative
    omegaQ = quaternion(0, omega(1), omega(2), omega(3));
    dq = compact(0.5 * q * omegaQ)';

    % Assemble
    dstate = [dpos; dq; dvel; domega];
end
```

**Dynamics notes:**
- Body-frame convention: x=forward, y=right, z=up.
- Gravity is projected into body frame via `R' * g_world` where `R` is body→world.
- `kTorque` is stored in `params.dynamics` with a default value of 0.01. This is the ratio of reaction torque to thrust. The sign pattern (+/-/ +/-) for rotors 1–4 ensures yaw torque cancels at equal thrust.
- Coriolis term `cross(omega, vel)` is included for full 6-DOF physical fidelity.
- No aerodynamic drag, ground effect, or blade-flapping model at this stage — those are Phase 09 refinements.

#### Method: `plot(ax)`

Override `AerialRobot.plot` (inherited via `Robot.plot`) to create wireframe from `buildGeometry`:

```matlab
function hg = plot(obj, ax)
    hg = plot@robot.Robot(obj, ax);
    [verts, faces, edges] = obj.buildGeometry();
    patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
          'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
    for i = 1:size(edges, 1)
        line('Parent', hg, ...
             'XData', verts(edges(i,1), 1), ...
             'YData', verts(edges(i,1), 2), ...
             'ZData', verts(edges(i,1), 3), ...
             'Color', 'k', 'LineWidth', 1.0);
    end
end
```

Same pattern as `DifferentialDrive.plot`. Slightly different `FaceColor` (`[0.8 0.8 0.9]` = light blue-grey) to visually distinguish aerial from ground robots.

### 3. Demo integration (`startRobot.m`)

The `startRobot.m` file (Phase 03) must be updated to add a `'Quadcopter'` case:

```matlab
case 'Quadcopter'
    params.geometry.armLength = 0.2;
    params.geometry.bodySize = [0.1, 0.1, 0.05];
    params.dynamics.mass = 0.5;
    params.dynamics.inertia = diag([0.002, 0.002, 0.004]);
    params.dynamics.maxThrust = 2.0;
    params.dynamics.maxTorque = 0.1;
    params.dynamics.kTorque = 0.01;
    robot.Quadcopter(params);
```

This is an additive change to `startRobot.m`; existing `'DifferentialDrive'` case is unchanged.

### 4. File structure

```
+robot/
  Robot.m               (Phase 01)
  AerialRobot.m         (Phase 04 — NEW)
  Quadcopter.m          (Phase 04 — NEW)
  DifferentialDrive.m   (Phase 02)
  DynamicsEngine.m      (Phase 02)
  Controller.m          (Phase 03)
  Direction.m           (Phase 01)
  Utils.m               (Phase 01)
  Visualizer.m          (Phase 01)
```

No changes to existing Phase 01–03 files except `startRobot.m` (additive case).

### 5. Verification criteria

A minimal validation script (`test_quadcopter.m`, placed under `.agent/skills/robot-dynamics/scripts/`) should verify:

1. **Geometry output:** `buildGeometry` returns N×3 verts, M×4 faces, P×2 edges with valid index ranges.
2. **Move commands:** each `move(direction, 1.0)` produces the expected thrust delta pattern from the table above.
3. **Hover:** `hover()` zeros the control vector.
4. **Forward dynamics:** an `UP` command (all rotors at hover thrust ~ mg/4) produces a stable near-zero vertical acceleration.
5. **Asymmetric thrust:** `ROLL_RIGHT` command produces positive roll acceleration (wx > 0).
6. **RK4 stability:** `step` with small dt (~0.001 s) maintains quaternion unit norm.
7. **Reset:** `reset()` restores `InitialState` and clears Control.

## Consequences

- `AerialRobot` provides a default `step()` with RK4 integration so all aerial subclasses get physics for free. The pattern mirrors `DifferentialDrive.step`.
- `hover()` is a convenience method with clear semantics — zero control — distinct from `STOP` (same effect, but `hover()` is named for the aerial domain).
- `Quadcopter` inherits from `AerialRobot`, not directly from `Robot`, keeping the hierarchy consistent with `docs/guide/architecture.md`.
- The full 6-DOF `computeDynamics` with coriolis terms is physically accurate enough for demonstration and education, while leaving room for aerodynamic refinements in Phase 09.
- The `kTorque` parameter is exposed in `params.dynamics`, making it tuneable without code changes.
- `startRobot.m` gets an additive `'Quadcopter'` case — no breaking changes to the existing DifferentialDrive flow.
- Phase 05 will add `GroundRobot` as a sibling of `AerialRobot` and re-parent `DifferentialDrive`, making the hierarchy complete.

## Files created

- `+robot/AerialRobot.m`
- `+robot/Quadcopter.m`
- `docs/adr/0003-quadcopter-and-aerialrobot.md`
