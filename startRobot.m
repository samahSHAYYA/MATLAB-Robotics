function startRobot(robotType)
    %STARTROBOT Entry point for the matlab-robotics demo.
    %   Creates one or more robots, a figure window, visualizer, and
    %   controller, then runs the interactive simulation loop.
    %
    %   startRobot('Quadruped')              Single robot
    %   startRobot({'Quadruped','Quadcopter'})  Multi-robot scene
    %
    %   Supported types: Quadruped, Quadcopter, DifferentialDrive, Humanoid
    %
    %   Controls during demo:
    %     arrows       Move/incline (momentary — move while held)
    %     w/s          Up/Down (aerial robots)
    %     a/d          Roll left/right (aerial robots)
    %     q/e          Pitch up/down (aerial robots)
    %     space        Stop/hover
    %     r            Reset to initial pose
    %     g            Toggle gait (quadruped / humanoid)
    %     h            Toggle HUD overlay
    %     c            Cycle camera mode
    %     l            Toggle running lights
    %     p            Cycle path mode (manual → record → replay)
    %     n            Cycle waypoint mode (off → place → navigate)
    %     tab          Cycle active robot (multi-robot)
    %     1-9          Jump to robot N (multi-robot)
    %     esc          Exit
    arguments
        robotType = 'DifferentialDrive'
    end

    import robot.DifferentialDrive
    import robot.Quadcopter
    import robot.Quadruped
    import robot.Humanoid
    import robot.Visualizer
    import robot.Controller

    if isstring(robotType) && isscalar(robotType)
        types = {robotType};
    elseif iscellstr(robotType) || isstring(robotType)
        types = cellstr(robotType);
    else
        error('robotType must be a string or cell array of strings.');
    end

    robots = cell(1, length(types));
    for k = 1:length(types)
        robots{k} = buildRobot(types{k});
    end

    nameList = strjoin(types, ' + ');
    delete(findall(groot, 'Type', 'figure'));
    fig = figure('Name', "matlab-robotics: " + nameList, ...
                 'NumberTitle', 'off', ...
                 'Color', 'white');

    ax = axes('Parent', fig);
    vis = Visualizer(ax);
    for k = 1:length(robots)
        vis.addRobot(robots{k});
    end
    drawnow;

    ctrl = Controller(fig, robots, vis);

    title(ax, char(nameList));

    nRobots = length(robots);
    if nRobots == 1
        fprintf('matlab-robotics demo: %s\n', types{1});
        fprintf('Controls: arrows=move, space=stop, r=reset, g=gait, h=HUD, c=camera, l=lights, p=path, n=waypoints, esc=exit\n');
    else
        fprintf('matlab-robotics demo: %s (%d robots)\n', nameList, nRobots);
        fprintf('Controls: arrows=move, tab=switch robot, 1-9=jump, space=stop, r=reset, g=gait, h=HUD, c=camera, l=lights, p=path, n=waypoints, esc=exit\n');
    end
    ctrl.run();
end

function rbt = buildRobot(type)
    switch type
        case 'DifferentialDrive'
            params.geometric.wheelRadius = 0.05;
            params.geometric.trackWidth  = 0.2;
            params.dynamic.mass        = 2.0;
            params.dynamic.inertia     = 0.05;
            params.dynamic.maxTorque   = 5.0;
            rbt = DifferentialDrive(params);
        case 'Quadcopter'
            params.geometric.armLength = 0.2;
            params.geometric.bodySize = [0.1, 0.1, 0.05];
            params.dynamic.mass = 0.5;
            params.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            params.dynamic.maxThrust = 2.0;
            params.dynamic.kTorque = 0.05;
            rbt = Quadcopter(params);
        case 'Quadruped'
            params.geometric.bodyLength = 0.4;
            params.geometric.bodyWidth = 0.2;
            params.geometric.bodyHeight = 0.1;
            params.kinematic.legLength1 = 0.15;
            params.kinematic.legLength2 = 0.15;
            params.geometric.shoulderWidth = 0.12;
            params.dynamic.mass = 3.0;
            params.dynamic.inertia = diag([0.015, 0.04, 0.05]);
            params.elastic.k_contact = 5000;
            params.elastic.b_contact = 50;
            params.elastic.mu = 0.8;
            rbt = Quadruped(params);
        case 'Humanoid'
            params.geometric.bodyHeight = 0.8;
            params.geometric.bodyWidth  = 0.4;
            params.geometric.hipWidth   = 0.2;
            params.kinematic.thighLength  = 0.35;
            params.kinematic.shinLength   = 0.35;
            params.kinematic.footLength   = 0.22;
            params.dynamic.mass = 30.0;
            params.dynamic.inertia = diag([0.5, 1.2, 1.0]);
            params.elastic.k_contact = 8000;
            params.elastic.b_contact = 80;
            params.elastic.mu = 0.9;
            params.balance.gainP = 1000;
            params.balance.gainD = 120;
            rbt = Humanoid(params);
        otherwise
            error('Unknown robot type: %s', type);
    end
end
