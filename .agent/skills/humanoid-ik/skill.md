# Humanoid IK Skill

## Overview

Humanoid IK for a bipedal robot with 2-link legs (thigh + shank) and feet.
Each leg has 3 DOF: hip yaw, hip roll, hip pitch, knee pitch (4 DOF per leg).

## Leg Geometry

```
   hip  ← body attachment point (hip joint)
    |\
    |  \  thighLength (L1)
    |    \
  knee ───  knee joint (pitch only)
    |
    |    shinLength (L2)
    |
  foot ───  ankle (2 DOF: roll + pitch)
```

## IK Convention

- Left leg: hip at `[-hipWidth/2, 0, 0]` in body frame
- Right leg: hip at `[+hipWidth/2, 0, 0]` in body frame
- Thigh rotates about knee pitch axis
- Shin rotates about ankle pitch axis
- Foot parallel to ground (ankle compensation)

## Control

Humanoid uses a 6-axis body wrench `[Fx, Fy, Fz, Tx, Ty, Tz]` like Quadruped.
Walking gait alternates left/right leg stance phases with:
- Double support phase (both feet on ground)
- Single support phase (one foot lifts and swings forward)
- Balance via torso lean and foot placement
