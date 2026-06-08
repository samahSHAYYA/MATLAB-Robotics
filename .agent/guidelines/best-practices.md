# Best Practices

## Naming conventions

- **Classes**: PascalCase — `Quadruped`, `DifferentialDrive`
- **Methods**: camelCase — `move`, `buildGeometry`, `computeDynamics`
- **Properties**: PascalCase — `Pose`, `State`, `Params`
- **Files**: match class name exactly — `Quadruped.m`
- **Constants**: UPPER_SNAKE_CASE — `GRAVITY`

## Documentation

- One-line docstring after `classdef` line and after each method signature.
- Math equations in comments where the code alone is unclear (quaternion derivatives, Jacobians).
- No inline comments for obvious statements.

## Code structure

- One class per file.
- Properties at top, then methods.
- Public methods before private.
- Use `methods (Access = private)` for internal helpers.

## MATLAB style

- 4-space indentation.
- No tabs.
- Max line length 100 characters.
- Use `arguments` blocks for input validation in public methods.
- Use `end` to close all blocks (classdef, properties, methods, function).

## Git

- One commit per completed phase.
- Commit message format: `phase-XX: brief description`.
- No force push, no amend.

## Testing

- Every robot class must have a companion test script under the skill's `scripts/`.
- Test scripts must be runnable standalone without user interaction.
- Physics tests should verify conservation of energy or momentum where applicable.
