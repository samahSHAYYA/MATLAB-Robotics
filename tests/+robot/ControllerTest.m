classdef ControllerTest < matlab.unittest.TestCase

    methods (Test)

        function constructorAcceptsFigureRobotVisualizer(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            r = robot.Quadcopter(p);
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            testCase.verifyTrue(ishandle(c.Figure));
            testCase.verifyTrue(c.Running);
            close(fig);
        end

        function setCommandForwardSetsRobotControl(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = struct();
            p.geometric.wheelRadius = 0.05;
            p.geometric.trackWidth = 0.2;
            p.dynamic.mass = 1.0;
            p.dynamic.inertia = 0.01;
            p.dynamic.maxTorque = 0.5;
            r = robot.DifferentialDrive(p);
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            c.setCommand(robot.Direction.FORWARD, 1);
            testCase.verifyEqual(r.Control, [r.maxTorque; r.maxTorque]);
            close(fig);
        end

        function setCommandStopZerosControl(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = struct();
            p.geometric.wheelRadius = 0.05;
            p.geometric.trackWidth = 0.2;
            p.dynamic.mass = 1.0;
            p.dynamic.inertia = 0.01;
            p.dynamic.maxTorque = 0.5;
            r = robot.DifferentialDrive(p);
            r.Control = [0.5; 0.5];
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            c.setCommand(robot.Direction.STOP, 0);
            testCase.verifyEqual(r.Control, [0; 0]);
            close(fig);
        end

        function setCommandStopZerosVelocity(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = struct();
            p.geometric.wheelRadius = 0.05;
            p.geometric.trackWidth = 0.2;
            p.dynamic.mass = 1.0;
            p.dynamic.inertia = 0.01;
            p.dynamic.maxTorque = 0.5;
            r = robot.DifferentialDrive(p);
            r.State(8:13) = [1; 0; 0; 0.5; 0; 0];
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            c.setCommand(robot.Direction.STOP, 0);
            testCase.verifyEqual(r.State(8:13), zeros(6, 1));
            close(fig);
        end

        function waypointsStartEmpty(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = paramsDiffDrive();
            r = robot.DifferentialDrive(p);
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            testCase.verifyEqual(size(c.Waypoints, 1), 0);
            testCase.verifyEqual(c.WaypointMode, "off");
            testCase.verifyEqual(c.WaypointTargetIdx, 1);
            close(fig);
        end

        function navigateToWaypointAdvancesTarget(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = paramsDiffDrive();
            r = robot.DifferentialDrive(p);
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            % Place robot at origin, waypoint at (0.5, 0, 0)
            r.setState([0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
            c.WaypointMode = "navigate";
            c.Waypoints = [0.5, 0, 0; 1.0, 0, 0];
            c.WaypointTargetIdx = 1;
            c.navigateToWaypoint();
            % Should yaw toward +X and move forward
            testCase.verifyTrue(norm(r.Control) > 0);
            % Step 40 times (2 seconds at 50 fps) to reach first waypoint
            for i = 1:80
                r.step(0, 0.025);
                c.navigateToWaypoint();
            end
            testCase.verifyEqual(c.WaypointTargetIdx, 2);
            close(fig);
        end

        function navigateToWaypointCompletes(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            p = paramsDiffDrive();
            r = robot.DifferentialDrive(p);
            v = robot.Visualizer(ax);
            c = robot.Controller(fig, r, v);
            r.setState([0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
            c.WaypointMode = "navigate";
            c.Waypoints = [0.1, 0, 0];
            c.WaypointTargetIdx = 1;
            c.navigateToWaypoint();
            r.step(0, 0.025);
            for i = 1:10
                r.step(0, 0.025);
                c.navigateToWaypoint();
            end
            % Should complete: WaypointMode goes back to "off"
            testCase.verifyEqual(c.WaypointMode, "off");
            close(fig);
        end
    end

end

function p = paramsDiffDrive()
    p.geometric.wheelRadius = 0.05;
    p.geometric.trackWidth  = 0.2;
    p.dynamic.mass        = 1.0;
    p.dynamic.inertia     = 0.01;
    p.dynamic.maxTorque   = 5.0;
end
