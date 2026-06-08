# Demo Guide

## Running

```matlab
% Add package to path (one time)
addpath('.');

% Launch a demo
demo('Quadruped')
demo('Quadcopter')
demo('DifferentialDrive')
```

The demo creates a figure, initializes the robot and visualizer, starts the keyboard-controlled real-time loop. Press **Esc** to exit.

## Controls

| Key | Action |
|---|---|
| Arrow keys | Move / rotate |
| W/S | Up/down (aerial) |
| A/D/E/Q | Roll/pitch (aerial) |
| Space | Stop |
| R | Reset |

## Extending

To add a new robot type:

1. Create a class in `+robot/` that inherits from `GroundRobot` or `AerialRobot`.
2. Implement `move()`, `buildGeometry()`, and `computeForces()`.
3. The demo system picks it up automatically if `demo('RobotName')` matches the class name.
4. Add keyboard mappings in `Controller` if new directions are needed.

## Debugging

- Run `robot.DynamicsEngine.rk4Step(...)` manually to test a single timestep.
- Set `Visualizer` to wireframe-only mode by passing `'wireframe'` style argument.
- Check state vector: `robot.getState()`.
- Plot leg workspace: `test_ik_3link` or `plot_leg_workspace` from quadruped-ik skill.
