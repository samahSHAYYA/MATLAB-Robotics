# Skill: Robot Dynamics

State and dynamics model for 6-DOF rigid-body robots in this project.

## State vector (13 elements)

```
state = [x; y; z; qw; qx; qy; qz; vx; vy; vz; wx; wy; wz]
```

| Index | Symbol | Description |
|---|---|---|
| 1–3 | x, y, z | Position in world frame (m) |
| 4–7 | qw, qx, qy, qz | Unit quaternion (world→body orientation) |
| 8–10 | vx, vy, vz | Linear velocity in body frame (m/s) |
| 11–13 | wx, wy, wz | Angular velocity in body frame (rad/s) |

## Control vector (varies by robot)

- **DifferentialDrive:** `[leftWheelTorque, rightWheelTorque]`
- **Quadcopter:** `[thrust1, thrust2, thrust3, thrust4]` (rotor forces, N)
- **Quadruped:** `[Fx, Fy, Fz, Tx, Ty, Tz]` (body-frame force + torque at CoM)

## ODE right-hand side (signature)

```matlab
function dstate = dynamics(obj, ~, state, control)
    % Extract state
    pos = state(1:3);
    q   = quaternion(state(4:7)');  % [w, x, y, z]
    vel = state(8:10);
    omega = state(11:13);

    % Rotation matrix from body to world
    R = rotmat(q, 'point');

    % Derivative of position: v in world frame
    dpos = R * vel;

    % Derivative of quaternion: dq = 0.5 * q * [0; omega]
    omegaQ = quaternion(0, omega(1), omega(2), omega(3));
    dq = compact(0.5 * q * omegaQ)';

    % Forces (compute in subclass-specific way)
    [F_body, T_body] = obj.computeForces(state, control);

    % Linear acceleration in body frame
    dvel = F_body / obj.mass - cross(omega, vel);

    % Angular acceleration (Euler equation)
    I = obj.inertia;
    domega = I \ (T_body - cross(omega, I * omega));

    % Add gravity projected into body frame
    g_world = [0; 0; -9.81];
    g_body = R' * g_world;
    dvel = dvel + g_body;

    dstate = [dpos; dq; dvel; domega];
end
```

## RK4 integration

```matlab
function nextState = rk4Step(dynFun, t, state, control, dt)
    k1 = dynFun(t, state, control);
    k2 = dynFun(t + dt/2, state + dt/2*k1, control);
    k3 = dynFun(t + dt/2, state + dt/2*k2, control);
    k4 = dynFun(t + dt, state + dt*k3, control);
    nextState = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
```

## Conventions

- Body frame: x=forward, y=right, z=up.
- Quaternion represents world→body rotation.
- Sub-stepping: physics dt = 1–5 ms, render dt = 16–33 ms (30–60 FPS).
- Ground contact: spring-damper penalty model at contact points.
