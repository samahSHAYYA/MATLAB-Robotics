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
Controller(fig, robot, visualizer)  → constructor, binds key handler (arrows, G toggles gait on Humanoid)
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

## `startRobot.m` (entry point)

```
startRobot()                              → default to Quadruped
startRobot('Quadruped')                   → quadruped with gait
startRobot('Quadcopter')                  → aerial 6-DOF
startRobot('DifferentialDrive')           → planar wheeled
startRobot('Humanoid')                    → bipedal walking
```

## `robot.GroundRobot` (abstract handle)

```
GroundRobot(params)           → constructor (3-DOF planar constraint)
step(t, dt)                   → RK4 integration (overrides Robot.step)
getControlDim()               → returns 2 (protected)
```

## `robot.AerialRobot` (abstract handle)

```
AerialRobot(params)           → constructor (full 6-DOF)
step(t, dt)                   → RK4 integration (overrides Robot.step)
hover()                       → zero control vector
```

## `robot.Humanoid` (handle)

```
Humanoid(params)             → constructor (bipedal, 6-DOF trunk dynamics)
move(direction, amount)       → 6-axis body wrench (same as Quadruped)
step(t, dt)                   → gait phase + IK + physics + wireframe
toggleGait()                  → enable/disable walking gait
reset()                       → restore initial conditions
buildGeometry()               → returns [verts, faces, edges] with foot boxes
computeDynamics(t, state, ctrl) → full 6-DOF with bipedal foot contact
plot(ax)                      → render wireframe in axes
getControlDim()               → returns 6
```

## `robot.Collision` (static methods)

```
checkOBB(centerA, quatA, halfA, centerB, quatB, halfB) → SAT overlap test
robotOBB(robot)               → get OBB center + half-size from robot
checkAll(robots, useParallel)  → pairwise collision matrix
buildOBB(robot)               → [8×3 vertices, 12×2 edges] world-frame OBB
```

## `RobotFleetApp` (multi-robot dashboard)

```
RobotFleetApp()                → open fleet command center (uifigure)
```

See `docs/adr/0013-robot-fleet-app.md` for architecture details.
