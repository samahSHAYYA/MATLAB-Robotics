# Architecture

## Class hierarchy

```
Robot (abstract, handle)
 ├── GroundRobot (abstract)    ← planar constraint, z=0, no roll/pitch
 │   ├── Quadruped              ← 3-link legs, IK, trot gait
 │   ├── DifferentialDrive      ← 2-wheel torque model
 │   └── Humanoid               ← bipedal 2-leg IK, walking gait
 └── AerialRobot (abstract)    ← full 6-DOF
     └── Quadcopter             ← 4-rotor thrust + torque
```

## State model

All dynamic robots use a 13-element state vector:

```
state = [x; y; z; qw; qx; qy; qz; vx; vy; vz; wx; wy; wz]
        |-- pos --|-- quat --|-- body vel --|-- body omega --|
```

- Position in world frame (m)
- Orientation as unit quaternion (world→body)
- Linear velocity in body frame (m/s)
- Angular velocity in body frame (rad/s)

## Dynamics

Each robot provides a `dynamics(t, state, control)` method returning the time derivative. `DynamicsEngine` integrates with RK4 and sub-stepping (1–5 ms physics dt).

Control vectors are robot-specific:

| Robot | Control | Dimension |
|---|---|---|
| DifferentialDrive | [leftWheelTorque, rightWheelTorque] | 2 |
| Quadcopter | [thrust1, thrust2, thrust3, thrust4] | 4 |
| Quadruped | [Fx, Fy, Fz, Tx, Ty, Tz] (body frame) | 6 |
| Humanoid  | [Fx, Fy, Fz, Tx, Ty, Tz] (body frame) | 6 |

## Visualization

- `hgtransform` parent for all wireframe geometry
- 4x4 transform matrix updated each frame (no object recreation)
- `drawnow limitrate` for non-blocking animation
- Simple ground plane grid for spatial reference

### Forward axis

All robots use +Y as the forward direction. For the Humanoid, the torso is proportioned with the deepest dimension along Z (height, 0.8 m), the second-largest along Y (forward depth, ~0.1 m), and the narrowest along X (lateral width, 0.4 m), matching human proportions.

## Control loop

```
while figure open
    read keyboard input → set control vector
    for each physics sub-step:
        dstate = robot.dynamics(t, state, control)
        state = rk4(dstate, dt_sub)
    visualizer.update(robot)
    drawnow limitrate
    t += dt_render
end
```

## RobotFleetApp

`RobotFleetApp` is a multi-robot dashboard built with `uifigure` and grid layouts. It manages up to 4 robots in a single 3D scene.

### UI layout

```
┌─────────────────────────────────────────────────┐
│ Title bar                                        │
├────────┬───────────────────────────┬─────────────┤
│ Spawn   │     Scene (3D axes)      │   Control   │
│ Panel   │                           │   Panel     │
│ [drop-  │   (single uiaxes with    │  [↑] [↺][↻] │
│  down]  │    hgtransform parent)   │  [←] [STOP] │
│ [+Spawn]│                           │  [→] [↓]   │
│ [+Cust] │                           │  [Formation]│
│ [-Remove]│                          │  [Reset]   │
│ [Script] │                          │             │
├────────┤                           ├─────────────┤
│ Legend  │                           │  Telemetry  │
│ ☑ vis ☐bbox R1                      │  Pos/Roll/  │
│ ☑ vis ☐bbox R2                      │  Pitch/Vel  │
├────────┴───────────────────────────┴─────────────┤
│ Status bar: [Status msg] [Pool] [FPS | Sim time] │
└─────────────────────────────────────────────────┘
```

### Spawn workflow

| Button | Behavior |
|--------|----------|
| `+ Spawn` | Creates robot at slot-based offset `x = -0.45*(n-1) + 0.675`, default Z from constructor. Id = `R{N}` (auto-incremented). |
| `+ Spawn (Custom...)` | Opens dialog with same defaults but user can rename and set exact position/orientation. |
| `- Remove Selected` | Deletes robot graphics, clears slot in `app.Robots` cell array. |

### Control routing

Commands flow through a `TargetDropdown` (ALL / R1–R4). In `simStep`:
1. `drawnow('limitrate')` flushes pending UI events (keyboard, buttons)
2. Target-matched robots receive `move(dir, amount)`
3. All active robots step physics (RK4 sub-steps)
4. Visible robots' `GraphicsTransform.Matrix` is updated

### Timer-based simulation

Uses `timer('ExecutionMode', 'fixedRate')` at `RenderDt` (≈33 ms). A `Busy` guard prevents re-entrant callbacks. `drawnow('limitrate')` at the start of each tick ensures UI callbacks (keyboard, checkboxes) are processed before physics.

### Parallel pool

Started asynchronously via a 0.5s single-shot timer — does not block UI construction. Status shown in the status bar: `Pool: 4w` when active, `Pool: N/A` when unavailable.
