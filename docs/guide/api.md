# API Reference

## `robot.Robot` (abstract handle)

```
Robot(params)             → constructor
move(direction, amount)   → abstract: apply control
buildGeometry()           → abstract: return [verts, faces] for wireframe
step(t, dt)               → integrate dynamics one timestep
reset()                   → restore initial conditions
plot(ax)                  → render wireframe in axes
getState() / setState()   → access state vector
```

## `robot.Direction` (enum)

```
FORWARD, BACKWARD, LEFT, RIGHT
UP, DOWN
YAW_LEFT, YAW_RIGHT
ROLL_LEFT, ROLL_RIGHT
PITCH_UP, PITCH_DOWN
STOP, RESET
```

## `robot.Visualizer`

```
Visualizer(ax)            → constructor, sets up axes, ground plane
addRobot(robot)           → create wireframe for a robot
update(robot)             → refresh wireframe transform
clear()                   → remove all robot graphics
```

## `robot.Controller`

```
Controller(fig, robot, visualizer)  → constructor, binds key handler
run()                               → start real-time control loop
stop()                              → exit the loop
setCommand(dir, amount)             → programmatic command (non-keyboard)
```

## `robot.DynamicsEngine`

```
DynamicsEngine(physicsDt)           → constructor with sub-step size
step(robot, t, dt)                  → integrate robot dynamics by dt
rk4Step(dynFun, t, state, control, dt) → static: single RK4 step
```

## `demo.m` (entry point)

```
demo()                              → default to Quadruped
demo('Quadruped')                   → robodog with gait
demo('Quadcopter')                  → aerial 6-DOF
demo('DifferentialDrive')           → planar wheeled
```
