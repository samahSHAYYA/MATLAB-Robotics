# matlab-robodog

6-DOF rigid-body dynamics simulation with wireframe visualization and keyboard control. Demonstrates robotics capability through three robot types: **Quadruped**, **Quadcopter**, and **DifferentialDrive**.

## Quick start

```matlab
demo('Quadruped')            % flagship — robodog with gait
demo('Quadcopter')           % aerial 6-DOF
demo('DifferentialDrive')    % planar wheeled
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
+robot/          ← MATLAB package (all classes)
.agent/          ← OpenCode agent system
docs/            ← ADRs + guide docs
resources/       ← images, schemas, 3D models
demo.m           ← entry point
opencode.json    ← agent/skill definitions
```

## System requirements

- MATLAB R2025 (or R2018b+ with quaternion support)
- No additional toolboxes required
