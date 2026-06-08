# ADR-0004: GroundRobot Abstraction

**Status:** accepted

**Date:** 2026-06-08

## Context

Phase 05 introduces `GroundRobot` as an abstract intermediate between `Robot` and ground-based concrete robots (`DifferentialDrive`, future `Quadruped`). The goal is to mirror the `AerialRobot` pattern established in ADR-0003 so that the hierarchy is symmetric:

```
Before:                    After:
Robot (abstract)           Robot (abstract)
 └── DifferentialDrive      ├── GroundRobot (abstract) ← NEW
                             │   └── DifferentialDrive ← REPARENTED
                             └── AerialRobot (abstract)
                                 └── Quadcopter
```

Three design forces:

- **Symmetry with AerialRobot:** `AerialRobot` provides a default `step()` with RK4 integration for all aerial subclasses. `GroundRobot` must provide the same for ground subclasses so that `DifferentialDrive` (and future `Quadruped`) inherit physics integration for free.
- **Control dimension variance:** Ground robots may have different control vector sizes. `DifferentialDrive` uses 2 (wheel torques), while `Quadruped` will use 8 (4 legs × 2 actuators). The `step()` method must handle a variable control dimension via an overridable helper.
- **No duplicate logic:** `DifferentialDrive.step` currently duplicates the RK4 integration pattern that also appears in `AerialRobot.step`. Pulling this into `GroundRobot.step` eliminates duplication for all ground robots.

## Decision

### 1. `+robot/GroundRobot.m` — Abstract base for ground robots

Inherits from `robot.Robot`. Mirrors the `AerialRobot` pattern with three differences:

1. **Control defaults to 2** (for DifferentialDrive) rather than 4 (for Quadcopter).
2. **`step()` uses `getControlDim()`** to allocate the zero vector, rather than hardcoding 4. This makes it subclass-safe for future robots with different control dimensions.
3. **No `hover()` method** — hover is an aerial concept. Ground robots use `move(STOP, 0)` instead.

#### Constructor

```matlab
function obj = GroundRobot(params)
    arguments
        params (1,1) struct
    end
    obj@robot.Robot(params);
    obj.Control = zeros(2, 1);
end
```

Calls the `Robot` base constructor with `params`, then initializes `Control` to a 2-element zero vector (the default for DifferentialDrive). Subclasses with a different control dimension (e.g., Quadruped with 8) override `getControlDim()` and set their own `Control` in their constructor.

#### `step(t, dt)` — RK4 integration

```matlab
function step(obj, t, dt)
    arguments
        obj
        t (1,1) double
        dt (1,1) double
    end
    if isempty(obj.Control)
        u = zeros(obj.getControlDim(), 1);
    else
        u = obj.Control;
    end
    dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
    s = robot.DynamicsEngine.rk4Step(dynFun, t, obj.State, u, dt);
    obj.setState(s);
end
```

Same pattern as `AerialRobot.step` but uses `getControlDim()` instead of a hardcoded `4`. This ensures correct behavior even when a subclass forgets to initialize `Control` in its constructor — the empty-check fallback produces a zero vector of the right size.

#### `getControlDim()` — Overridable helper

```matlab
methods (Access = protected)
    function n = getControlDim(obj)
        n = 2;
    end
end
```

Protected so it is not part of the public interface. Subclasses override this to match their control vector size (e.g., Quadruped returns 8).

#### Semantically abstract

`GroundRobot` is not MATLAB-abstract (no new `methods (Abstract)` beyond those inherited from `Robot`). It is "semantically abstract": it should not be instantiated directly, but this is enforced by convention, not language restriction. The guard against direct instantiation is that `GroundRobot` does not implement `move`, `buildGeometry`, or `computeDynamics` — any attempt to call them will error.

#### Design rationale

- **Protected `getControlDim`** rather than a public property: ensures subclasses cannot accidentally change the dimension mid-simulation. Overriding a method is explicit and intentional.
- **`step()` lives in `GroundRobot`** rather than staying duplicated in each concrete class: eliminates a cross-cutting maintenance burden. If the integrator changes (e.g., from RK4 to something else in a future phase), only `GroundRobot.step` needs updating.
- **Empty-control fallback** mirrors `AerialRobot.step`: if a subclass constructor forgets to set `Control`, the fallback produces a zero vector of the correct dimension rather than propagating an empty matrix into `computeDynamics`.

### 2. Changes to `+robot/DifferentialDrive.m`

Three changes, all mechanical:

1. **Re-parent:** `classdef DifferentialDrive < robot.Robot` → `classdef DifferentialDrive < robot.GroundRobot`
2. **Constructor call:** `obj@robot.Robot(params)` → `obj@robot.GroundRobot(params)`
3. **Remove `step()` override:** Now inherited from `GroundRobot`. The implementation is identical except the empty-control fallback uses `getControlDim()` instead of hardcoded `[0; 0]` — `getControlDim()` returns `2` for DifferentialDrive, so behavior is unchanged.

Everything else — `properties`, `move`, `buildGeometry`, `computeDynamics`, `plot` — remains identical.

### 3. No changes to `demo.m` or `Controller.m`

The re-parenting is transparent to consumers. `demo('DifferentialDrive')` continues to work because `DifferentialDrive` is still `isa(robot.Robot)` (through `GroundRobot ← Robot`). No consumer code checks for `DifferentialDrive` by its parent class name.

## Consequences

- **Symmetric hierarchy** — `GroundRobot` mirrors `AerialRobot` at the same level. The class tree is now:
  ```
  Robot
   ├── GroundRobot (abstract)
   │   ├── DifferentialDrive
   │   └── Quadruped (future)
   └── AerialRobot (abstract)
       └── Quadcopter
  ```
- **One less `step()` override** — `DifferentialDrive` shrinks by ~15 lines. Future ground robots (Quadruped) get physics integration for free.
- **`getControlDim()` pattern** — establishes a hook for variable control dimensions without resorting to properties that could be mutated at runtime.
- **No breaking changes** — the re-parenting is invisible to all existing code that consumes `DifferentialDrive` through the `Robot` interface. `isa(obj, 'robot.Robot')` and `isa(obj, 'robot.DifferentialDrive')` checks both work correctly.
- **ADR-0003 forward reference resolved** — ADR-0003 noted that Phase 05 would add `GroundRobot` and re-parent `DifferentialDrive`. This ADR completes that work.

## Files created

- `+robot/GroundRobot.m`
- `docs/adr/0004-ground-robot-abstraction.md`

## Files modified

- `+robot/DifferentialDrive.m` — re-parented from `Robot` to `GroundRobot`, constructor call updated, `step()` override removed.
