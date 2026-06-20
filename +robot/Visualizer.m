classdef Visualizer < handle
    %VISUALIZER  3D scene manager: axes, ground plane, robot transforms.
    %   Manages a single set of 3D axes with perspective view, a ground
    %   patch, and an hgtransform container for each robot's graphics.
    properties
        AxesHandle      (1,1) matlab.graphics.axis.Axes
        TransformGroup  (1,1) matlab.graphics.primitive.Transform
        Robots                 cell
        GroundHandle    (1,1) matlab.graphics.primitive.Patch
        CameraMode      (1,1) string = "free"
        WaypointMarkers      cell  = {}
        WaypointLineHandle
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
            obj.WaypointLineHandle = line(ax, NaN, NaN, NaN, ...
                'Color', [1 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--');
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
            if ismethod(rbt, 'updateVisuals')
                rbt.updateVisuals(obj.AxesHandle);
            end
        end

        function clear(obj)
            %CLEAR  Remove all robot graphics and clear the robot list.
            delete(obj.TransformGroup.Children);
            obj.Robots = {};
        end

        function addWaypointMarker(obj, pos, idx)
            %ADDWAYPOINTMARKER  Place a 3D marker at the given position.
            ax = obj.AxesHandle;
            hold(ax, 'on');
            m = scatter3(ax, pos(1), pos(2), pos(3), 60, ...
                [1 0.6 0], 'filled', 'MarkerEdgeColor', 'k', ...
                'LineWidth', 1.0);
            t = text(ax, pos(1), pos(2), pos(3)+0.05, num2str(idx), ...
                'FontSize', 9, 'FontWeight', 'bold', ...
                'Color', [1 0.6 0], 'HorizontalAlignment', 'center');
            obj.WaypointMarkers{end+1} = {m, t};
            obj.refreshWaypointLine();
        end

        function clearWaypoints(obj)
            for i = 1:length(obj.WaypointMarkers)
                pair = obj.WaypointMarkers{i};
                if ishandle(pair{1}); delete(pair{1}); end
                if ishandle(pair{2}); delete(pair{2}); end
            end
            obj.WaypointMarkers = {};
            set(obj.WaypointLineHandle, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end

        function highlightWaypoint(obj, idx)
            for i = 1:length(obj.WaypointMarkers)
                pair = obj.WaypointMarkers{i};
                if ~ishandle(pair{1}); continue; end
                if i < idx
                    set(pair{1}, 'MarkerFaceColor', [0.3 0.8 0.3]);
                    set(pair{2}, 'Color', [0.3 0.8 0.3]);
                elseif i == idx
                    set(pair{1}, 'MarkerFaceColor', [1 0 0], 'SizeData', 100);
                    set(pair{2}, 'Color', [1 0 0], 'FontSize', 11);
                else
                    set(pair{1}, 'MarkerFaceColor', [1 0.6 0], 'SizeData', 60);
                    set(pair{2}, 'Color', [1 0.6 0], 'FontSize', 9);
                end
            end
        end

        function refreshWaypointLine(obj)
            n = length(obj.WaypointMarkers);
            if n < 2
                set(obj.WaypointLineHandle, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                return;
            end
            x = zeros(n, 1); y = zeros(n, 1); z = zeros(n, 1);
            for i = 1:n
                p = obj.WaypointMarkers{i}{1};
                x(i) = p.XData; y(i) = p.YData; z(i) = p.ZData;
            end
            set(obj.WaypointLineHandle, 'XData', x, 'YData', y, 'ZData', z);
        end
    end
end
