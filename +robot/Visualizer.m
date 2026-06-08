classdef Visualizer < handle
    properties
        AxesHandle      (1,1) matlab.graphics.axis.Axes
        TransformGroup  (1,1) matlab.graphics.primitive.Transform
        Robots                 cell
        GroundHandle    (1,1) matlab.graphics.primitive.Patch
    end

    methods
        function obj = Visualizer(ax)
            arguments
                ax (1,1) matlab.graphics.axis.Axes
            end
            obj.AxesHandle = ax;
            hold(ax, 'on');
            axis(ax, 'equal');
            grid(ax, 'on');
            view(ax, 3);
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
            xlim(ax, [-5, 5]);
            ylim(ax, [-5, 5]);
            zlim(ax, [-5, 5]);

            obj.TransformGroup = hgtransform(ax);

            gx = [-5, -5, 5, 5];
            gy = [-5, 5, 5, -5];
            gz = [0, 0, 0, 0];
            obj.GroundHandle = patch(ax, gx, gy, gz, [0.85, 0.85, 0.85]);
        end

        function addRobot(obj, robot)
            robot.plot(obj.TransformGroup);
            obj.Robots{end + 1} = robot;
        end

        function update(obj, robot)
            R = Utils.quatToRotmx(robot.State(4:7));
            pos = robot.State(1:3);
            T = eye(4);
            T(1:3, 1:3) = R;
            T(1:3, 4) = pos;
            robot.GraphicsTransform.Matrix = T;
        end

        function clear(obj)
            delete(obj.TransformGroup.Children);
            obj.Robots = {};
        end
    end
end
