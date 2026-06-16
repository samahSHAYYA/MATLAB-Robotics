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
| 01 | 2026-06-08 | Direction, Utils, Robot (abstract), Visualizer |
| 02 | 2026-06-08 | DifferentialDrive + DynamicsEngine |
| 03 | 2026-06-08 | Controller + startRobot.m |
| 04 | 2026-06-08 | Quadcopter + AerialRobot |
| 05 | 2026-06-08 | GroundRobot abstraction |
| 06 | 2026-06-08 | Quadruped body + leg geometry |
| 07 | 2026-06-08 | Quadruped IK + leg animation |
| 08 | 2026-06-08 | Quadruped trot gait + keyboard control |
| 09 | 2026-06-08 | Polish: physics, visuals, edge cases |
| 10 | 2026-06-16 | Humanoid body dynamics + leg geometry |
| 11 | 2026-06-16 | Humanoid IK + bipedal walking gait |
| 12 | 2026-06-16 | Humanoid keyboard control + integration |
