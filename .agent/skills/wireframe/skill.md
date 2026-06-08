# Skill: Wireframe Visualization

How to render robot wireframes using MATLAB `hgtransform` and `patch`.

## Core approach

1. Create an `hgtransform` parent object in the axes.
2. Add `patch` or `line` children for geometry.
3. Update the `Matrix` property each frame (faster than recreating objects).

```matlab
ax = gca;
hold(ax, 'on');
view(3);
axis equal;
grid on;

tform = hgtransform(ax);

% Draw a box as wireframe
verts = [0 0 0; L 0 0; L W 0; 0 W 0; 0 0 H; L 0 H; L W H; 0 W H];
faces = [1 2 3 4; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1; 5 6 7 8];
patch('Parent', tform, 'Vertices', verts, 'Faces', faces, ...
      'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 1.5);
```

## Frame update

```matlab
% Build 4x4 transform matrix from position quaternion
R = rotmat(q, 'point');
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = position;
tform.Matrix = T;

drawnow limitrate;
```

## Performance rules

- Use `hgtransform` — do not `cla()` and redraw each frame.
- Use `drawnow limitrate` — `drawnow` blocks and destroys framerate.
- Pre-allocate all graphics objects during setup.
- For complex robots, group sub-assemblies under child transforms.

## Ground plane

```matlab
patch([-5 -5 5 5], [-5 5 5 -5], [0 0 0 0], [0.9 0.9 0.9]);
```

## Scripts

- `scripts/test-wireframe.m` — verifies hgtransform + patch rendering renders correctly.
- `scripts/test-drawnow.m` — validates animation loop does not drop below 30 FPS.
