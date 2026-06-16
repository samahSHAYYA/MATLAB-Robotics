# matlab-robotics

6-DOF rigid-body dynamics simulation with wireframe visualization and keyboard control. Demonstrates robotics capability through four robot types: **Quadruped**, **Quadcopter**, **DifferentialDrive**, and **Humanoid**.

## Quick start

```matlab
startRobot('Quadruped')            % flagship — quadruped with gait
startRobot('Quadcopter')           % aerial 6-DOF
startRobot('DifferentialDrive')    % planar wheeled
startRobot('Humanoid')             % bipedal walking
```

## Documentation

| Document | Description |
|---|---|
| `guide/architecture.md` | Class hierarchy, state model, dynamics |
| `guide/interaction.md`  | Keyboard map, agent handoff protocol |
| `guide/api.md`          | Public interface per class |
| `guide/demo-guide.md`   | How to run, extend, and debug |
| `guide/references.md`   | Dependencies, resources, version info |
| `adr/`                  | Architecture Decision Records |

## Project structure

```
+robot/          ← MATLAB package (source)
.agent/          ← OpenCode agent system
docs/            ← ADRs + guide docs
qa/              ← QA verification scripts (local only, gitignored)
resources/       ← images, schemas, 3D models
tests/           ← Unit tests (mirrors +robot/ package structure)
startRobot.m     ← entry point
opencode.json    ← agent/skill definitions
```

## Running tests

```matlab
addpath('tests');
runtests('tests')
```

## System requirements

- MATLAB R2025 (or R2018b+ with quaternion support)
- No additional toolboxes required
