function test_keymap
% TEST_KEYMAP Validate keyboard mapping produces correct Direction values

    import robot.Direction;

    % Simulate MATLAB KeyPress event data structures
    keys = {'uparrow', 'downarrow', 'leftarrow', 'rightarrow', ...
            'w', 's', 'a', 'd', 'q', 'e', 'space', 'r'};
    expected = {Direction.FORWARD, Direction.BACKWARD, Direction.YAW_LEFT, Direction.YAW_RIGHT, ...
                Direction.UP, Direction.DOWN, Direction.ROLL_LEFT, Direction.ROLL_RIGHT, ...
                Direction.PITCH_UP, Direction.PITCH_DOWN, Direction.STOP, Direction.RESET};

    passes = 0;
    total = length(keys);

    for i = 1:total
        result = mapKey(keys{i});
        if result == expected{i}
            passes = passes + 1;
            fprintf('[PASS] %s -> %s\n', keys{i}, char(result));
        else
            fprintf('[FAIL] %s -> %s (expected %s)\n', keys{i}, char(result), char(expected{i}));
        end
    end

    fprintf('\n%d / %d passed.\n', passes, total);
    if passes == total
        fprintf('[PASS] All key mappings correct.\n');
    else
        fprintf('[FAIL] Some mappings incorrect.\n');
    end
end

function dir = mapKey(key)
    import robot.Direction;
    switch key
        case 'uparrow',     dir = Direction.FORWARD;
        case 'downarrow',   dir = Direction.BACKWARD;
        case 'leftarrow',   dir = Direction.YAW_LEFT;
        case 'rightarrow',  dir = Direction.YAW_RIGHT;
        case 'w',           dir = Direction.UP;
        case 's',           dir = Direction.DOWN;
        case 'a',           dir = Direction.ROLL_LEFT;
        case 'd',           dir = Direction.ROLL_RIGHT;
        case 'q',           dir = Direction.PITCH_UP;
        case 'e',           dir = Direction.PITCH_DOWN;
        case 'space',       dir = Direction.STOP;
        case 'r',           dir = Direction.RESET;
        otherwise,          error('Unknown key: %s', key);
    end
end
