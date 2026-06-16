# ADR-0005: Quadruped Trunk Dynamics and Leg Geometry

**Status:** accepted

**Date:** 2026-06-08

## Context

Phase 06 must deliver the flagship robot: a 4-legged (quadruped) with full 6-DOF trunk dynamics and wireframe leg geometry. It is the most complex robot in the project because it combines:

- **6-DOF rigid-body trunk** — same class of dynamics as Quadcopter (ADR-0003), but controlled by body-frame force/torque rather than rotor thrusts.
- **Ground contact** — 4 feet interact with the ground via a penalty-based spring-damper model. This is the first contact dynamics in the project (Quadcopter has no contact; DifferentialDrive uses no-slip kinematic constraints).
- **Leg geometry** — 3-link legs (hip abduction + hip flexion + knee flexion) visualized as 2-segment lines. No active joint actuation in this phase — legs are passive geometry that follows the trunk.
- **Hierarchical placement** — inherits from `GroundRobot` (ADR-0004) which provides RK4 `step()` via `getControlDim()`. The control dimension for body-force control is 6, not 2 (DifferentialDrive) or 4 (Quadcopter). This is the first ground robot with 6-DOF trunk dynamics.

Three design forces:

1. **Control abstraction level:** The control vector should be body-frame force/torque (`[Fx; Fy; Fz; Tx; Ty; Tz]`), not individual joint torques. Joint-level actuation (IK + leg servos) is Phase 07–08. This keeps Phase 06 focused on trunk physics and contact.
2. **Ground contact model:** Needs to be simple (penalty-based) but sufficient to prevent trunk penetration and produce plausible foot forces. Friction should be included to prevent lateral sliding.
3. **Default stance simplicity:** Without active gait (Phase 08), feet are fixed in a default vertical stance below each shoulder. No IK computation is needed — foot positions are derived directly from geometry parameter `legLength1 + legLength2`.

## Decision

### 1. `+robot/Quadruped.m` — Concrete 6-DOF ground robot

Inherits from `robot.GroundRobot`. The class is placed at the same level as `DifferentialDrive` in the hierarchy:

```
Robot
 ├── GroundRobot (abstract)
 │   ├── DifferentialDrive    (3-DOF planar, 2 control)
 │   └── Quadruped            (6-DOF spatial, 6 control) ← NEW
 └── AerialRobot (abstract)
     └── Quadcopter           (6-DOF spatial, 4 control)
```

#### Params struct

```matlab
params.geometry.bodyLength     % trunk length (m), default 0.4
params.geometry.bodyWidth      % trunk width (m), default 0.2
params.geometry.bodyHeight     % trunk height (m), default 0.1
params.geometry.legLength1     % upper leg (m), default 0.15
params.geometry.legLength2     % lower leg (m), default 0.15
params.geometry.shoulderWidth  % half-width between shoulder joints (m), default 0.12
params.dynamics.mass           % total trunk mass (kg), default 3.0
params.dynamics.inertia        % 3x3 inertia matrix (kg*m^2)
params.dynamics.k_contact      % ground contact stiffness (N/m), default 1000
params.dynamics.b_contact      % ground contact damping (N*s/m), default 10
params.dynamics.mu             % Coulomb friction coefficient, default 0.8
```

#### Properties

Unpacked from `Params` into typed instance properties for fast access in the hot dynamics loop — same pattern as `DifferentialDrive` and `Quadcopter`:

```matlab
properties
    bodyLength    (1,1) double
    bodyWidth     (1,1) double
    bodyHeight    (1,1) double
    legLength1    (1,1) double
    legLength2    (1,1) double
    shoulderWidth (1,1) double
    mass          (1,1) double
    inertia       (3,3) double
    k_contact     (1,1) double
    b_contact     (1,1) double
    mu            (1,1) double
end
```

#### State

Standard 13-element vector (same as all `Robot` subclasses). All 6 DOF are active:

| Index | Field | Frame |
|---|---|---|
| 1–3 | x, y, z | World position (m) |
| 4–7 | qw, qx, qy, qz | Unit quaternion, world→body |
| 8–10 | vx, vy, vz | Body-frame linear velocity (m/s) |
| 11–13 | wx, wy, wz | Body-frame angular velocity (rad/s) |

#### Control

6-element column vector: `[Fx; Fy; Fz; Tx; Ty; Tz]` representing net body-frame force and torque applied at the center of mass.

This is a **direct force/torque abstraction** — the controller commands the net wrench on the trunk, not individual leg forces. This matches Phase 06 scope (trunk dynamics only). Phase 07+ will convert leg joint torques into a net wrench via the Jacobian.

#### `getControlDim()` override

```matlab
methods (Access = protected)
    function n = getControlDim(obj)
        n = 6;
    end
end
```

The constructor explicitly sets `obj.Control = zeros(6, 1)` after the `GroundRobot` constructor call (which initialises to `zeros(2, 1)`).

#### Method: `move(direction, amount)`

Maps `Direction` to body-frame force/torque. `amount` is clamped to `[0, 1]` and scaled by `maxForce` (for forces) and `maxTorque` (for torques). These scaling factors are derived from `mass * g` and `inertia` respectively.

| Direction | Control effect |
|---|---|
| `FORWARD` | `[+F; 0; 0; 0; 0; 0]` |
| `BACKWARD` | `[-F; 0; 0; 0; 0; 0]` |
| `LEFT` | `[0; +F; 0; 0; 0; 0]` (strafe left) |
| `RIGHT` | `[0; -F; 0; 0; 0; 0]` (strafe right) |
| `UP` | `[0; 0; +F; 0; 0; 0]` (hop/leap) |
| `DOWN` | `[0; 0; -F; 0; 0; 0]` (push down) |
| `YAW_LEFT` | `[0; 0; 0; 0; 0; +T]` |
| `YAW_RIGHT` | `[0; 0; 0; 0; 0; -T]` |
| `ROLL_LEFT` | `[0; 0; 0; +T; 0; 0]` |
| `ROLL_RIGHT` | `[0; 0; 0; -T; 0; 0]` |
| `PITCH_UP` | `[0; 0; 0; 0; +T; 0]` |
| `PITCH_DOWN` | `[0; 0; 0; 0; -T; 0]` |
| `STOP` | `[0; 0; 0; 0; 0; 0]` |
| `RESET` | Calls `obj.reset()` |

**Semantics:**
- The control vector is set **absolutely** (not additive), unlike Quadcopter's additive delta pattern. This is because body-frame force/torque is a direct command, not integrated from rotor increments.
- `STOP` zeros the control vector — the robot coasts under gravity and contact forces.
- `RESET` restores `InitialState` and clears `Control`.
- Forces are signed per the body frame convention (x=forward, y=right, z=up).

#### Method: `buildGeometry()`

Returns `[verts, faces, edges]` for a wireframe quadruped:

**Body:** Rectangular prism (box) centered at origin with dimensions `bodyLength × bodyWidth × bodyHeight`. Same 8-vertex, 6-face, 12-edge pattern as `DifferentialDrive.buildGeometry`.

**Legs:** 4 legs, each with 2 segments:
- Upper leg: shoulder→knee (line or thin box)
- Lower leg: knee→foot (line or thin box)

Shoulder positions in body frame (trunk-coxa joints):
- FL (front-left):  `[+bodyLength/2, +shoulderWidth, 0]`
- FR (front-right): `[+bodyLength/2, -shoulderWidth, 0]`
- HL (hind-left):   `[-bodyLength/2, +shoulderWidth, 0]`
- HR (hind-right):  `[-bodyLength/2, -shoulderWidth, 0]`

Default stance (passive, vertical): knee is at shoulder position offset by `[0, 0, -legLength1]`, foot at `[0, 0, -(legLength1 + legLength2)]` in the leg base frame. In body frame:
- Knee: `[shoulder_x, shoulder_y, -legLength1]`
- Foot: `[shoulder_x, shoulder_y, -(legLength1 + legLength2)]`

Each leg is drawn as two line segments (shoulder→knee, knee→foot) with a small sphere/marker at the foot tip to indicate the contact point.

**Edge layout:**
```
nBody = 8 vertices for body box
+ 4 knee vertices
+ 4 foot vertices
= 16 vertices total

Edges:
  bodyEdges: 12 box edges
  legEdges:  8 edges (2 per leg: shoulder→knee, knee→foot)
  = 20 edges total
```

Faces are only defined for the body box (6 quad faces). Legs are drawn as edges only.

#### Method: `computeDynamics(t, state, control)`

Full 6-DOF rigid-body dynamics (identical structure to `Quadcopter.computeDynamics`) with the addition of spring-damper ground contact forces at each foot:

```matlab
function dstate = computeDynamics(obj, t, state, control)
    % --- Extract state ---
    q = quaternion(state(4:7)');
    vel = state(8:10);
    omega = state(11:13);
    R = rotmat(q, 'point');           % body → world

    % --- Gravity in body frame ---
    g_world = [0; 0; -9.81];
    g_body = R' * g_world;

    % --- Control forces (body frame) ---
    F_control = control(1:3);
    T_control = control(4:6);

    % --- Ground contact forces (accumulated across 4 feet) ---
    F_contact_body = [0; 0; 0];       % sum of foot contact forces in body frame
    T_contact = [0; 0; 0];            % sum of foot contact torques in body frame

    % Shoulder positions in body frame
    sFL = [ obj.bodyLength/2;  obj.shoulderWidth; 0];
    sFR = [ obj.bodyLength/2; -obj.shoulderWidth; 0];
    sHL = [-obj.bodyLength/2;  obj.shoulderWidth; 0];
    sHR = [-obj.bodyLength/2; -obj.shoulderWidth; 0];
    shoulders = [sFL, sFR, sHL, sHR];

    for i = 1:4
        % Foot position in body frame (default vertical stance)
        foot_body = shoulders(:,i) + [0; 0; -(obj.legLength1 + obj.legLength2)];

        % Foot position in world frame
        foot_world = R * foot_body + state(1:3);

        % Foot velocity in world frame
        J = [eye(3), -R * robot.Utils.skew(foot_body)];
        foot_vel_world = J * [vel; omega];

        % Ground penetration check
        if foot_world(3) < 0
            penetration = -foot_world(3);
            penetration_vel = -foot_vel_world(3);

            % Spring-damper normal force (world frame, positive up)
            Fn = obj.k_contact * penetration + obj.b_contact * penetration_vel;
            Fn = max(0, Fn);    % no suction

            % Coulomb friction (simple)
            v_horiz = foot_vel_world(1:2);
            v_horiz_norm = norm(v_horiz);
            if v_horiz_norm > 1e-6
                mu_applied = obj.mu;
                Ff = -mu_applied * Fn * (v_horiz / v_horiz_norm);
            else
                Ff = [0; 0];
            end

            F_contact_world = [Ff(1); Ff(2); Fn];

            % Accumulate in body frame
            F_contact_body = F_contact_body + R' * F_contact_world;
            T_contact = T_contact + cross(foot_body, R' * F_contact_world);
        end
    end

    % --- Total body force ---
    F_total_body = F_control + g_body * obj.mass + F_contact_body;

    % --- Linear acceleration (body frame) ---
    dvel = F_total_body / obj.mass - cross(omega, vel);

    % --- Angular acceleration (Euler equation) ---
    I = obj.inertia;
    domega = I \ (T_control + T_contact - cross(omega, I * omega));

    % --- Position derivative (world frame) ---
    dpos = R * vel;

    % --- Quaternion derivative ---
    omegaQ = quaternion(0, omega(1), omega(2), omega(3));
    dq = compact(0.5 * q * omegaQ)';

    % --- Assemble ---
    dstate = [dpos; dq; dvel; domega];
end
```

**Contact model details:**
- **Normal force:** Penalty-based spring-damper. `F_n = k * p + b * p_dot` where `p` is penetration depth. Force is applied upward (positive z). Clamped to non-negative (no adhesive suction).
- **Friction:** Coulomb approximation. Tangential force opposes horizontal velocity with magnitude `mu * F_n`. No stiction model — this is a dynamic friction-only approximation, sufficient for demonstration.
- **Foot Jacobian:** `J = [I_3, -R * skew(r_foot_body)]` maps body-frame velocity to world-frame foot velocity. The skew term accounts for rotational velocity at the foot tip.

#### Method: `plot(ax)`

Overrides `GroundRobot.plot` (inherited from `Robot.plot`). Creates wireframe from `buildGeometry`:

```matlab
function hg = plot(obj, ax)
    hg = plot@robot.Robot(obj, ax);
    [verts, faces, edges] = obj.buildGeometry();
    % Body as filled patch
    patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
          'FaceColor', [0.7 0.8 0.7], 'EdgeColor', 'none');
    % Legs and body edges as lines
    for i = 1:size(edges, 1)
        line('Parent', hg, ...
             'XData', verts(edges(i,:), 1), ...
             'YData', verts(edges(i,:), 2), ...
             'ZData', verts(edges(i,:), 3), ...
             'Color', 'k', 'LineWidth', 1.5);
    end
end
```

FaceColor `[0.7 0.8 0.7]` (light green) to visually distinguish quadruped from DifferentialDrive (grey, `[0.7 0.7 0.7]`) and Quadcopter (blue-grey, `[0.8 0.8 0.9]`).

#### Method: `reset()`

Restores `InitialState` and clears control. Inherits from `Robot.reset()` — no override needed unless additional state (e.g., gait phase) is added in Phase 07+.

### 2. Initial state and default pose

The trunk starts at `[0; 0; bodyHeight/2 + legLength1 + legLength2]` — i.e., standing with feet touching the ground (z=0) and trunk elevated by the leg length plus half the body height. Orientation is identity (level, facing +x). All velocities start at zero.

### 3. Demarcation from future phases

Phase 06 delivers the trunk dynamics + passive leg geometry + contact model. It explicitly does **not** include:

- **Joint-level actuation** (Phase 07) — legs are rigid links that follow the trunk; no joint torques or servo models.
- **Inverse kinematics** (Phase 07) — default stance uses vertical legs; no IK computation.
- **Gait patterns** (Phase 08) — no trot, walk, or other footfall sequencing. Feet remain fixed below shoulders.
- **Joint limits** (Phase 07) — no angle or velocity limits on hip/knee joints.

These are deferred because each requires significant design and testing. The ADR boundary ensures trunk physics and contact can be validated independently before adding leg articulation.

### 4. Edge index offset strategy

`buildGeometry` must return vertex indices in `faces` and `edges` that correctly reference rows of the concatenated `verts` array. The standard pattern (used in `DifferentialDrive.buildGeometry` and `Quadcopter.buildGeometry`) is:

```
verts = [bodyVerts; legVerts]
faces = [bodyFaces]   % only body has faces; legs are edges only
edges = [bodyEdges; legEdges + nBody]
```

Each leg contributes 2 edges (shoulder→knee, knee→foot). The 4 legs produce 8 leg edges.

### 5. Demo integration

The `startRobot.m` file (Phase 03) gains a `'Quadruped'` case:

```matlab
case 'Quadruped'
    params.geometry.bodyLength = 0.4;
    params.geometry.bodyWidth = 0.2;
    params.geometry.bodyHeight = 0.1;
    params.geometry.legLength1 = 0.15;
    params.geometry.legLength2 = 0.15;
    params.geometry.shoulderWidth = 0.12;
    params.dynamics.mass = 3.0;
    params.dynamics.inertia = diag([0.01, 0.02, 0.015]);
    params.dynamics.k_contact = 1000;
    params.dynamics.b_contact = 10;
    params.dynamics.mu = 0.8;
    robot.Quadruped(params);
```

This is additive — existing `'DifferentialDrive'` and `'Quadcopter'` cases are unchanged.

## Consequences

- **New class in hierarchy** — `Quadruped < GroundRobot < Robot`. Gets RK4 `step()` for free via inherited `GroundRobot.step`.
- **Control dimension 6** — `getControlDim()` returns 6, matching the body-frame force/torque control vector. This is the first `GroundRobot` subclass to use a non-2 control dimension.
- **Contact dynamics via penalty** — spring-damper ground contact is simple but introduces a stiff ODE. The RK4 integrator with `dt = 1e-3` should maintain stability for `k_contact = 1000`; higher stiffness may require smaller timesteps or an implicit integrator (Phase 09).
- **Friction model is simple** — Coulomb friction without stiction means the robot will slide on slopes below the friction cone angle. This is acceptable for demonstration. A stiction model (e.g., Brown's model or a regularised Coulomb) can be added in Phase 09 if needed.
- **Legs are passive geometry** — `buildGeometry` returns fixed leg positions relative to the body frame. When the trunk rotates, legs move with it through the `hgtransform.Matrix` in `Visualizer.update`. This is correct for Phase 06 but will need updating when Phase 07 adds IK-driven leg animation.
- **No joint angle state** — Phase 06 stores no leg configuration in `State`. If Phase 07 adds joint angles as state elements, the state vector will grow from 13 to 13 + 4×3 = 25, requiring a state dimension override (see ADR-0001, state is not overridable yet — will need an ADR for variable-length state).
- **Demo case added** — existing demo flow for DifferentialDrive and Quadcopter is unaffected.
- `buildGeometry` returns 16 vertices, 6 faces, 20 edges. This is lightweight enough for real-time animation.

## Files created

- `+robot/Quadruped.m`
- `docs/adr/0005-quadruped-trunk-and-leg-geometry.md`

## Files modified

- `startRobot.m` — additive `'Quadruped'` case (delegated to implementation phase)
