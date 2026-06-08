# References

## MATLAB version

- Target: **MATLAB R2025**
- Minimum: R2018b (built-in `quaternion` type availability)
- No additional toolboxes required

## Dependencies

- No external libraries or packages
- Core MATLAB graphics (`hgtransform`, `patch`, `drawnow`)
- Built-in `quaternion` class (R2018b+)

## Resources

| Resource | Location |
|---|---|
| Project code | `+robot/` |
| Agent system | `.agent/` |
| Architecture decisions | `docs/adr/` |
| Schemas | `resources/schemas/` |
| 3D models | `resources/models/` |
| Images | `resources/images/` |

## Reference literature

- Quadruped IK based on standard 3-link analytic IK approaches
- Quadcopter dynamics using standard rotor mixing matrix
- DifferentialDrive using unicycle-model with wheel torque control
- Contact model: spring-damper penalty method

## Version history

| Phase | Date | Change |
|---|---|---|
| 00 | 2026-06-08 | Initial scaffold |
