classdef VisualizerTest < matlab.unittest.TestCase

    methods (Test)

        function constructorAcceptsAxes(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            v = robot.Visualizer(ax);
            testCase.verifyEqual(v.AxesHandle, ax);
            testCase.verifyTrue(ishandle(v.TransformGroup));
            testCase.verifyTrue(ishandle(v.GroundHandle));
            close(fig);
        end

        function addRobotStoresGraphics(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            v = robot.Visualizer(ax);
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            r = robot.Quadcopter(p);
            v.addRobot(r);
            testCase.verifyEqual(numel(v.Robots), 1);
            testCase.verifyTrue(ishandle(r.GraphicsTransform));
            close(fig);
        end

        function updateDoesNotError(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            v = robot.Visualizer(ax);
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            r = robot.Quadcopter(p);
            v.addRobot(r);
            v.update(r);
            testCase.verifyEqual(v.Robots{1}, r);
            close(fig);
        end

        function clearRemovesGraphics(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            v = robot.Visualizer(ax);
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            r = robot.Quadcopter(p);
            v.addRobot(r);
            v.clear();
            testCase.verifyEmpty(v.Robots);
            testCase.verifyTrue(isempty(v.TransformGroup.Children));
            close(fig);
        end

    end

end
