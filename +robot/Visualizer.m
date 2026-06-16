classdef Visualizer < handle
    %VISUALIZER  3D scene manager: axes, ground plane, robot transforms.
    %   Manages a single set of 3D axes with perspective view, a ground
    %   patch, and an hgtransform container for each robot's graphics.
    properties
        AxesHandle      (1,1) matlab.graphics.axis.Axes
        TransformGroup  (1,1) matlab.graphics.primitive.Transform
        Robots                 cell
        GroundHandle    (1,1) matlab.graphics.primitive.Patch
    end

    methods
        function obj = Visualizer(ax)
            %VISUALIZER  Construct visualizer and set up the 3D scene.
            %   vis = robot.Visualizer(ax)
            %   Inputs: ax - matlab.graphics.axis.Axes handle
            %   Sets equal aspect, perspective view, grid, labels, limits,
            %   a parent hgtransform, and a grey ground patch at z=0.
            arguments
                ax (1,1) matlab.graphics.axis.Axes
            end
            obj.AxesHandle = ax;
            hold(ax, 'on');
            axis(ax, 'equal');
            grid(ax, 'on');
            view(ax, 3);
            ax.Projection = 'perspective';
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
            xlim(ax, [-1.5, 1.5]);
            ylim(ax, [-1.5, 1.5]);
            zlim(ax, [-0.5, 2.0]);

            obj.TransformGroup = hgtransform(ax);

            gx = [-5, -5, 5, 5];
            gy = [-5, 5, 5, -5];
            gz = [0, 0, 0, 0];
            obj.GroundHandle = patch(ax, gx, gy, gz, [0.85, 0.85, 0.85]);
        end

        function addRobot(obj, rbt)
            %ADDROBOT  Add a robot to the scene and create its graphics.
            %   Inputs: rbt - robot.Robot subclass instance
            %   Calls rbt.plot() to build the visual representation.
            rbt.plot(obj.TransformGroup);
            obj.Robots{end + 1} = rbt;
        end

        function update(obj, rbt)
            %UPDATE  Update the robot's 4×4 transform from its current state.
            %   Computes R = quatToRotmx(State(4:7)), extracts position
            %   from State(1:3), and applies the result to the robot's
            %   GraphicsTransform.Matrix for wireframe animation.
            R = robot.Utils.quatToRotmx(rbt.State(4:7));
            pos = rbt.State(1:3);
            T = eye(4);
            T(1:3, 1:3) = R;
            T(1:3, 4) = pos;
            rbt.GraphicsTransform.Matrix = T;
        end

        function clear(obj)
            %CLEAR  Remove all robot graphics and clear the robot list.
            delete(obj.TransformGroup.Children);
            obj.Robots = {};
        end
    end
end
