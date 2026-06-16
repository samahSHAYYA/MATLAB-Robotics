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

    end

end
