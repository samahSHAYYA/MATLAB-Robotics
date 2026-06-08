# Interaction

## Keyboard map

| Key | Action | Affects |
|---|---|---|
| ↑ | FORWARD (body +x) | All |
| ↓ | BACKWARD (body -x) | All |
| ← | YAW_LEFT (rotate -z) | All |
| → | YAW_RIGHT (rotate +z) | All |
| W | UP (world +z) | Aerial only |
| S | DOWN (world -z) | Aerial only |
| A | ROLL_LEFT (rotate -x) | Aerial only |
| D | ROLL_RIGHT (rotate +x) | Aerial only |
| Q | PITCH_UP (rotate -y) | Aerial only |
| E | PITCH_DOWN (rotate +y) | Aerial only |
| Space | STOP / HOVER | All |
| R | RESET initial pose | All |
| Esc | Close demo | All |

Ground robots (Quadruped, DifferentialDrive) ignore aerial-only commands silently.

## Direction enum

```matlab
classdef Direction
    enumeration
        FORWARD, BACKWARD, LEFT, RIGHT
        UP, DOWN
        YAW_LEFT, YAW_RIGHT
        ROLL_LEFT, ROLL_RIGHT
        PITCH_UP, PITCH_DOWN
        STOP, RESET
    end
end
```

## Agent handoff protocol

See `.agent/guidelines/workflow.md` for full details. Summary:

1. Orchestrator delegates to Architect for design
2. Architect produces ADR and design spec
3. Orchestrator delegates to Developer for implementation
4. Developer loads skill, reads ADR, writes code
5. Orchestrator delegates to QA for verification
6. QA runs tests, reports results
7. Orchestrator updates tasks.json and commits
