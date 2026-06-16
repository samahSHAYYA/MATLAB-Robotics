# matlab-robotics

[![MATLAB](https://img.shields.io/badge/MATLAB-R2025%2B-orange)](https://www.mathworks.com/products/matlab.html)
[![CI](.github/workflows/ci.yml)](.github/workflows/ci.yml)

6-DOF rigid-body dynamics simulation with wireframe visualization and keyboard 
control.

Supports **Quadruped**, **Quadcopter**, **DifferentialDrive**, and **Humanoid** robots.

## Quick start

```matlab
addpath('.');
startRobot('Quadruped')
```

## Documentation

See [docs/README.md](docs/README.md) for full architecture, API, interaction, 
and demo guides. All design decisions are recorded as ADRs in 
[docs/adr/](docs/adr/).

## Project structure

```
+robot/          MATLAB package (source)
.agent/          Agent system (orchestrator, architect, developer, QA)
docs/            ADRs and guide documentation
qa/              QA verification scripts (local only, gitignored)
resources/       Assets (images, schemas, 3D models)
tests/           Unit tests (mirrors +robot/ package structure)
```

## Running tests

```matlab
addpath('tests');
runtests('tests')
```

## Requirements

MATLAB R2025 (or R2018b+). No additional toolboxes.
