function demo(robotType)
    arguments
        robotType string = 'DifferentialDrive'
    end

    import robot.*

    switch robotType
        case 'DifferentialDrive'
            params.geometry.wheelRadius = 0.05;
            params.geometry.trackWidth  = 0.2;
            params.dynamics.mass        = 2.0;
            params.dynamics.inertia     = 0.05;
            params.dynamics.maxTorque   = 5.0;
            robot = DifferentialDrive(params);
        otherwise
            error('Unknown robot type: %s', robotType);
    end

    fig = figure('Name', ['matlab-robodog: ' robotType], ...
                 'NumberTitle', 'off', ...
                 'Color', 'white', ...
                 'KeyPressFcn', []);

    ax = axes('Parent', fig);
    vis = Visualizer(ax);
    vis.addRobot(robot);

    ctrl = Controller(fig, robot, vis);
    title(ax, robotType);

    fprintf('matlab-robodog demo: %s\n', robotType);
    fprintf('Controls: arrows=move, space=stop, r=reset, esc=exit\n');
    ctrl.run();
end
