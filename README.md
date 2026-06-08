# matlab-robodog

6-DOF rigid-body dynamics simulation with wireframe visualization and keyboard control.

Supports **Quadruped**, **Quadcopter**, and **DifferentialDrive** robots.

## Quick start

```matlab
addpath('.');
demo('Quadruped')
```

## Documentation

See [docs/README.md](docs/README.md) for full architecture, API, interaction, and demo guides. All design decisions are recorded as ADRs in [docs/adr/](docs/adr/).

## Project structure

```
+robot/          MATLAB package
.agent/          Agent system (orchestrator, architect, developer, QA)
docs/            ADRs and guide documentation
resources/       Assets (images, schemas, 3D models)
```

## Requirements

MATLAB R2025 (or R2018b+). No additional toolboxes.
