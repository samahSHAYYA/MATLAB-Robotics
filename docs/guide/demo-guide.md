# Demo Guide

## Running

```matlab
% Add package to path (one time)
addpath('.');

% Launch a demo
startRobot('Quadruped')
startRobot('Quadcopter')
startRobot('DifferentialDrive')
startRobot('Humanoid')
```

The demo creates a figure, initializes the robot and visualizer, starts the keyboard-controlled real-time loop. Press **Esc** to exit.

## Controls

| Key | Action |
|---|---|
| Arrow keys | Move / rotate (forward axis = +Y) |
| W/S | Up/down (aerial) |
| A/D/E/Q | Roll/pitch (aerial) |
| G | Toggle gait on/off (Humanoid only) |
| Space | Stop |
| R | Reset |

**Humanoid** walks forward/backward with ↑/↓, turns (yaw) with ←/→, and toggles gait with `G`. Forward is +Y (torso depth axis).

## Extending

To add a new robot type:

1. Create a class in `+robot/` that inherits from `GroundRobot` or `AerialRobot`.
2. Implement `move()`, `buildGeometry()`, and `computeForces()`.
3. The demo system picks it up automatically if `startRobot('RobotName')` matches the class name.
4. Add keyboard mappings in `Controller` if new directions are needed.

## Debugging

- Run `robot.DynamicsEngine.rk4Step(...)` manually to test a single timestep.
- Set `Visualizer` to wireframe-only mode by passing `'wireframe'` style argument.
- Check state vector: `robot.getState()`.
- For Humanoid leg IK debugging, inspect foot positions and contact forces in the dynamics loop.
