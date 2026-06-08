# Skill: Keyboard Control

How to capture keyboard input and drive robot movement in real time.

## WindowKeyPressFcn

Use `WindowKeyPressFcn` on the figure (not `KeyPressFcn`) — it captures all keys regardless of UI focus.

```matlab
fig.WindowKeyPressFcn = @(src, evt) onKeyPress(src, evt, robot);
```

## Key to direction mapping

```matlab
function onKeyPress(~, evt, robot)
    switch evt.Key
        case 'uparrow'
            robot.move(robot.Direction.FORWARD, 1.0);
        case 'downarrow'
            robot.move(robot.Direction.BACKWARD, 1.0);
        case 'leftarrow'
            robot.move(robot.Direction.YAW_LEFT, 0.5);
        case 'rightarrow'
            robot.move(robot.Direction.YAW_RIGHT, 0.5);
        case 'w'
            robot.move(robot.Direction.UP, 1.0);
        case 's'
            robot.move(robot.Direction.DOWN, 1.0);
        case 'a'
            robot.move(robot.Direction.ROLL_LEFT, 0.5);
        case 'd'
            robot.move(robot.Direction.ROLL_RIGHT, 0.5);
        case 'q'
            robot.move(robot.Direction.PITCH_UP, 0.5);
        case 'e'
            robot.move(robot.Direction.PITCH_DOWN, 0.5);
        case 'space'
            robot.move(robot.Direction.STOP, 0);
        case 'r'
            robot.reset();
    end
end
```

## Real-time control loop

```matlab
function runLoop(robot, visualizer, fig)
    t = 0;
    while ishandle(fig)
        dt = 0.02;  % Fixed timestep (50 Hz physics)
        robot.step(t, dt);  % Integrate dynamics
        visualizer.update(robot);
        drawnow limitrate;
        t = t + dt;
    end
end
```

## Design notes

- The `step(t, dt)` method on Robot internally calls `DynamicsEngine.rk4Step()`.
- The key press sets the `control` vector on the robot; step reads and uses it.
- To avoid key-repeat issues, store the desired control and let step consume it.

## Scripts

- `scripts/test-keymap.m` — validates key press produces correct Direction enum value.
