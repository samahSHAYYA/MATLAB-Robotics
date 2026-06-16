> **Status: Historical reference.** This document describes an aspirational
> multi-robot vision that has not been implemented. The current project
> focuses on individual robot demos (Humanoid, Quadruped, Quadcopter,
> DifferentialDrive). See [CHANGELOG](../CHANGELOG.md) and
> [tasks.json](../.agent/tasks.json) for actual progress.

# matlab-robotics — Vision & Architecture

## Overview

Extend the single-robot keyboard demo into a multi-robot simulation
environment with command sequencing, coordinate-frame visualization,
trajectory recording, and interactive scene control.

---

## 1.  Multi-Robot Scene

### 1.1  Spawning
- `startRobot` accepts a cell array of {type, params} pairs:
  ```matlab
  robots = {
    'Quadcopter',       params_qc;
    'Quadruped',        params_qp;
    'DifferentialDrive' params_dd
  };
  startRobot(robots);
  ```
- Each robot runs its own physics (independent state, dynamics, control).
- All robots share the same 3D axes, ground plane, lighting.

### 1.2  Physics Loop
- Single global time `t` advances by `dt` each frame.
- ALL robots step together (same sub-steps, same dt).
- ALL robots update their visual transform each frame.

```
for each render frame:
    process keyboard
    move(activeRobot, direction, amount)    ← only active robot
    for substep = 1:N
        sequencer.dispatch(t)               ← timed commands
        for each robot
            robot.step(t, dt)
        t += dt
    for each robot
        visualizer.update(robot)
    for each robot
        trailRecorder.record(robot)
    render axes, trails, UI
```

---

## 2.  Active Robot & Switching

### 2.1  Selecting a Robot
| Key | Action |
|---|---|
| Tab | Cycle active robot forward |
| Shift+Tab | Cycle backward |
| 1 - 9 | Jump to robot N directly |

### 2.2  Visual Indicator
- Active robot gets a coloured bounding box or highlighted nose.
- Title bar shows: `"Active: Quadcopter [2/3]"`.
- Inactive robots render at half brightness or grey tint.

### 2.3  Keyboard Routing
- All Direction commands route to the active robot only.
- Global keys (Esc, Space, R) apply to active robot.
- Per-robot keys (G for gait toggle) apply to active robot.

---

## 3.  Command Sequencing

### 3.1  Data Model

```
ScheduledCommand
    Time        double          trigger time (seconds from epoch)
    RobotIdx    double          1-based robot index
    Direction   robot.Direction motion command
    Amount      double          [0, 1] authority
```

A sequence is a `ScheduledCommand` array sorted by `Time`.

### 3.2  CommandSequencer

```
CommandSequencer < handle
    Commands        ScheduledCommand  sorted by time
    DispatchIdx     double            next un-dispatched index
    RecordedLog     ScheduledCommand  live-captured log
    IsRecording     logical

    add(cmd)                        insert sorted
    dispatch(t)                     fire all commands with Time ≤ t
    record(robotIdx, dir, amt, t)   capture live command
    startRecording() / stopRecording()
    saveSequence(path)
    loadSequence(path) → commands
    reset()                         rewind DispatchIdx
```

### 3.3  Dispatch Rules
- `dispatch(t)` fires every pending command whose `Time ≤ t`.
- Fired commands are consumed (not re-fired on re-dispatch).
- `reset()` rewinds `DispatchIdx` to 1 (replay from start).

### 3.4  Recording
- Keystrokes are captured with current simulation time `t`.
- Stored as `ScheduledCommand` in `RecordedLog`.
- Can be saved as `.mat` and reloaded.

### 3.5  Modes
| Mode | Behaviour |
|---|---|
| Live | Keyboard → active robot. No sequencer dispatch. |
| Playback | Sequencer dispatches. Keyboard still works for active robot (overrides). |
| Record | Live keyboard + capture to RecordedLog. |
| Offline | No keyboard. Pure sequencer dispatch. |

Toggle with key `p` (cycle: live → playback → record → live).

---

## 4.  Coordinate Frames

### 4.1  World Frame
- RGB axes at world origin: X=red, Y=green, Z=blue.
- Arrow heads at tips.
- Labels: "X", "Y", "Z".
- Length: configurable (default 0.5 m).

### 4.2  Body Frame (per robot)
- Same RGB axes attached to each robot's `GraphicsTransform`.
- Moves and rotates with the robot.
- Smaller scale (e.g. 0.15 m).

### 4.3  Toggle
- Key `x` toggles world frame visibility.
- Key `z` toggles body frame visibility.
- Both on by default.

### 4.4  Implementation
- A lightweight class `FrameAxes` that holds 3 lines + 3 text objects.
- World axes: direct child of scene axes.
- Body axes: child of robot's hgtransform (moves automatically).

---

## 5.  Trajectory Visualization

### 5.1  TrailRecorder

```
TrailRecorder < handle
    MaxPoints       double     default 2000
    Trails          cell       {robotIdx → N×3 trail points}
    TrailLines      cell       {robotIdx → line handle}
    IsVisible       logical    default true
    TrailColor      {robotIdx → RGB}

    record(robot)               append current position to trail
    toggleVisibility()
    clear()
    setColor(robotIdx, rgb)
```

### 5.2  Rendering
- Trail is a `line()` object with fading alpha or color gradient.
- Each robot gets its own trail colour (auto-assigned from a palette).
- Trail updates every render frame (not every physics sub-step).

### 5.3  Controls
| Key | Action |
|---|---|
| `t` | Toggle trail visibility on/off |
| `T` (Shift+t) | Clear all trails |

---

## 6.  Simulation Controls

### 6.1  Pause / Resume
- Key `p` (or `space` when no direction active) toggles pause.
- Frozen display, no physics, no sequencer dispatch.

### 6.2  Speed Control
- Keys `[` / `]` decrease/increase simulation speed.
- Range: 0.1× to 10×.
- Display current speed in title bar.

### 6.3  Status Display
- Title bar or text overlay shows:
  - Simulation time
  - Active robot name + index
  - Mode (Live / Playback / Record)
  - Speed multiplier

---

## 7.  Class Dependencies

```
                    ┌──────────────┐
                    │  startRobot  │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
       ┌───────────┐ ┌──────────┐ ┌──────────┐
       │ MultiCtrl │ │Visualizer│ │Sequencer │
       └─────┬─────┘ └────┬─────┘ └────┬─────┘
             │            │            │
    ┌────────┴───┐  ┌─────┴──────┐    │
    ▼            ▼  ▼            ▼    │
┌───────┐ ┌──────────┐ ┌──────────┐   │
│Robot[]│ │FrameAxes │ │TrailRec  │   │
└───────┘ └──────────┘ └──────────┘   │
                                       ▼
                                ┌──────────────┐
                                │ScheduledCmd  │
                                └──────────────┘
```

---

## 8.  Sprint Breakdown

### Sprint 1 — Multi-Robot Core
- [ ] `MultiController` class (multi-robot render loop)
- [ ] `startRobot.m` accepts robot array
- [ ] Active robot switching (Tab, number keys)
- [ ] Visual indicator for active robot
- [ ] Keyboard routes to active robot

### Sprint 2 — Command Sequencing
- [ ] `ScheduledCommand` data class
- [ ] `CommandSequencer` (insert, dispatch, reset)
- [ ] Record mode (capture keystrokes with timestamps)
- [ ] Save / load sequence .mat
- [ ] Playback mode

### Sprint 3 — Coordinate Frames
- [ ] `FrameAxes` class (world frame)
- [ ] Body frame per robot (child of hgtransform)
- [ ] Toggle visibility (x / z keys)

### Sprint 4 — Trajectory Visualization
- [ ] `TrailRecorder` (append, line handle, fading)
- [ ] Per-robot trail colours
- [ ] Toggle / clear trails (t / T keys)

### Sprint 5 — Simulation Controls & Polish
- [ ] Pause / resume
- [ ] Speed control ([ / ] keys)
- [ ] Status display (time, active robot, mode, speed)
- [ ] Timeline scrubber (basic slider)
- [ ] Export video (MATLAB `VideoWriter`)

### Sprint 6 — UI Panel & Sensors (stretch)
- [ ] In-figure UI panel (sliders, buttons)
- [ ] Per-robot runtime parameter editing
- [ ] Collision detection between robots
- [ ] Sensor cones / rays

---

## 9.  File Map (after implementation)

```
+robot/
    MultiController.m       multi-robot render loop + robot switching
    CommandSequencer.m      timed command dispatch / recording
    ScheduledCommand.m      unit command data class
    TrailRecorder.m         pose history logging + trail line rendering
    FrameAxes.m             3D axis arrows (world + body)
    Controller.m            kept as-is for single-robot use
    Visualizer.m            extended: multi-robot update, trail support
    Robot.m                 no change
    AerialRobot.m           no change
    GroundRobot.m           no change
    Quadcopter.m            no change
    Quadruped.m             no change
    DifferentialDrive.m     no change
    Direction.m             no change
    DynamicsEngine.m        no change
    Utils.m                 no change

startRobot.m                updated: accepts robot array or single

docs/
    VISION.md               this file
    SPRINTS.md              sprint tracking (tasks per sprint)
```

---

## 10.  Design Principles

1. **Backward compatible** — `startRobot('Quadcopter')` still works.
2. **Single responsibility** — each class does one thing.
3. **Deterministic replay** — same sequence → same behaviour.
4. **Minimal coupling** — sequencer, trails, axes don't depend on each other.
5. **Toggle everything** — every visual overlay has a show/hide key.
