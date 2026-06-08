# Skill: Quadruped IK

Analytic inverse kinematics for a 3-link quadruped leg.

## Leg configuration (3 DOF)

Each leg has three joints:
1. **Hip abduction** (shoulder yaw) — rotates leg out from body, axis Z
2. **Hip flexion** (shoulder pitch) — moves upper leg forward/back, axis Y
3. **Knee flexion** (elbow pitch) — moves lower leg, axis Y

Link lengths are stored in `params.leg.L1` (upper leg), `params.leg.L2` (lower leg).

## Frame conventions

- Leg base frame: x=forward, y=outward, z=down (relative to body)
- Hip abduction rotates about z
- Hip flexion rotates about y of resulting frame
- Knee flexion rotates about y of resulting frame

## Analytic IK

Given foot position `[x, y, z]` in the leg base frame:

```matlab
function [theta1, theta2, theta3] = legIK(x, y, z, L1, L2)
    % Hip abduction angle
    theta1 = atan2(y, z);  % or atan2(y, -z) depending on sign convention

    % Distance from hip to foot in the leg plane
    r = sqrt(x^2 + y^2 + z^2);

    % Knee angle (law of cosines)
    cos_theta3 = (L1^2 + L2^2 - r^2) / (2 * L1 * L2);
    cos_theta3 = max(-1, min(1, cos_theta3));  % clamp
    theta3 = acos(cos_theta3);

    % Hip flexion angle
    theta2 = atan2(x, sqrt(y^2 + z^2)) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
end
```

## Workspace constraints

- Knee should not hyperextend: `theta3` in [0.3, 2.8] rad
- Hip abduction limited: `theta1` in [-0.5, 0.5] rad
- Legs should not cross body midline

## Trotting gait

Simple trot pattern: diagonal leg pairs swing synchronously.

```
Pair 1: FR (front-right) + HL (hind-left)  — swing phase
Pair 2: FL (front-left) + HR (hind-right)  — stance phase
```

Foot trajectory is a half-ellipse in the leg frame during swing, fixed during stance.

## Scripts

- `scripts/test-ik-3link.m` — verifies IK produces valid angles for a known foot position.
- `scripts/plot-leg-workspace.m` — visualizes reachable workspace.
