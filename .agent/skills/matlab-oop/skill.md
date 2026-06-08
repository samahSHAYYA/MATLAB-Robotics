# Skill: MATLAB OOP

Key facts for writing MATLAB classes in `+robot/` package.

## Handle vs Value

- Robot classes must inherit `handle` for mutable state (pose, velocity).
- Use `classdef MyClass < handle`.

## Abstract methods

```matlab
classdef Robot < handle
    methods (Abstract)
        move(obj, direction, amount)
        buildGeometry(obj)
    end
end
```

## Package convention

- `+robot/` directory → `import robot.*` in consuming code.
- Fully qualified access: `robot.Quadruped`, `robot.Direction`.
- Package directory must start with `+`.

## Properties

```matlab
properties
    Pose   (1,1) struct   % .position [3x1], .orientation [3x1] (RPY)
    Params (1,1) struct
end
```

## Built-in quaternion type (R2018b+)

```matlab
q = quaternion(w, x, y, z);
qRot = rotmat(q, 'frame');
```

## Graphics

- `hgtransform` as parent for wireframe objects.
- Set `Matrix` property to a 4x4 transform.
- `drawnow limitrate` for animation (not `drawnow`).
- `WindowKeyPressFcn` for keyboard capture.

## Scripts

- `scripts/scaffold-robot.m` — generates a new robot class from template.
- `scripts/validate-class.m` — checks class inherits handle and implements all abstract methods.
