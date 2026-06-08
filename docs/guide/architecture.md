# Architecture

## Class hierarchy

```
Robot (abstract, handle)
 ├── GroundRobot (abstract)    ← planar constraint, z=0, no roll/pitch
 │   ├── Quadruped              ← 3-link legs, IK, trot gait
 │   └── DifferentialDrive      ← 2-wheel torque model
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

## Visualization

- `hgtransform` parent for all wireframe geometry
- 4x4 transform matrix updated each frame (no object recreation)
- `drawnow limitrate` for non-blocking animation
- Simple ground plane grid for spatial reference

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
