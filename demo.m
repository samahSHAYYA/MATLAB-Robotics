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
        case 'Quadcopter'
            params.geometry.armLength = 0.2;
            params.geometry.bodySize = [0.1, 0.1, 0.05];
            params.dynamics.mass = 0.5;
            params.dynamics.inertia = diag([0.002, 0.002, 0.004]);
            params.dynamics.maxThrust = 2.0;
            params.dynamics.kTorque = 0.01;
            robot = Quadcopter(params);
        case 'Quadruped'
            params.geometry.bodyLength = 0.4;
            params.geometry.bodyWidth = 0.2;
            params.geometry.bodyHeight = 0.1;
            params.geometry.legLength1 = 0.15;
            params.geometry.legLength2 = 0.15;
            params.geometry.shoulderWidth = 0.12;
            params.dynamics.mass = 3.0;
            params.dynamics.inertia = diag([0.01, 0.02, 0.015]);
            params.dynamics.k_contact = 1000;
            params.dynamics.b_contact = 10;
            params.dynamics.mu = 0.8;
            robot = Quadruped(params);
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
