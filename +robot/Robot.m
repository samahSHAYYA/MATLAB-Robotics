classdef Robot < handle
    %ROBOT  Abstract base class for all rigid-body robots.
    %   Subclasses must implement move(), buildGeometry(), and
    %   computeDynamics().  State layout:
    %     State(1:3)   — position [x; y; z] in world frame
    %     State(4:7)   — orientation as unit quaternion [w; x; y; z]
    %     State(8:10)  — velocity in body frame [vx; vy; vz]
    %     State(11:13) — angular velocity in body frame [wx; wy; wz]
    properties
        Id                  (1,1) string = ""
        Pose                (1,1) struct
        State               (13,1) double
        Control             (:,1) double
        InitialState        (13,1) double
        InitialControl      (:,1) double
        GraphicsTransform
        TrailBuffer         (:,3) double = zeros(0,3)
        TrailMaxLen         (1,1) double = 300
        TrailHandle
        ShadowHandle
        LightsOn            (1,1) logical = false
        RunningLightHandles        cell = {}
    end

    methods (Abstract)
        %MOVE  Apply a motion command from the keyboard controller.
        %   Inputs: direction - robot.Direction enum, amount - [0,1] scalar
        move(obj, direction, amount)

        %BUILDGEOMETRY  Return wireframe vertices, faces, edges.
        %   [verts, faces, edges] = buildGeometry(obj)
        [verts, faces, edges] = buildGeometry(obj)

        %COMPUTEDYNAMICS  Compute state derivative at current condition.
        %   dstate = computeDynamics(obj, t, state, control)
        %   Inputs:  t       - time (s)
        %            state   - 13×1 state vector
        %            control - control input vector
        %   Outputs: dstate  - 13×1 state derivative
        dstate = computeDynamics(obj, t, state, control)
    end

    methods
        function obj = Robot()
            %ROBOT  Construct base robot with zero state and identity pose.
            obj.State = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            obj.InitialState = obj.State;
            obj.Control = [];
            obj.InitialControl = obj.Control;
            obj.Pose = struct('position', [0; 0; 0], 'orientation', [0; 0; 0]);
            obj.updatePoseFromState();
        end

        function step(obj, ~, ~)
            arguments
                obj
                ~
                ~
            end
            obj.updatePoseFromState();
            obj.recordTrailPoint();
        end

        function reset(obj)
            %RESET  Restore state and control to initial conditions.
            obj.State = obj.InitialState;
            obj.Control = obj.InitialControl;
            obj.TrailBuffer = zeros(0, 3);
            if ~isempty(obj.TrailHandle) && ishandle(obj.TrailHandle)
                set(obj.TrailHandle, 'XData', [], 'YData', [], 'ZData', []);
            end
            obj.updatePoseFromState();
        end

        function hg = plot(obj, ax)
            %PLOT  Create an hgtransform container for robot wireframe.
            %   hg = plot(obj, ax) attaches a new hgtransform to axes ax
            %   and stores it in obj.GraphicsTransform.
            %   Outputs: hg - hgtransform handle
            hg = hgtransform(ax);
            obj.GraphicsTransform = hg;
        end

        function s = getState(obj)
            %GETSTATE  Return the full 13×1 state vector.
            s = obj.State;
        end

        function setState(obj, s)
            %SETSTATE  Set the state vector and update derived pose.
            %   Inputs: s - 13×1 [pos; quat; vel; omega]
            arguments
                obj
                s (13,1) double
            end
            obj.State = s;
            obj.updatePoseFromState();
            obj.recordTrailPoint();
        end
    end

    methods
        function updateVisuals(obj, ~)
            %UPDATEVISUALS  Refresh trail line, shadow, and running lights.
            if ~isempty(obj.TrailHandle) && ishandle(obj.TrailHandle)
                buf = obj.TrailBuffer;
                if size(buf, 1) >= 2
                    set(obj.TrailHandle, ...
                        'XData', buf(:,1), 'YData', buf(:,2), 'ZData', buf(:,3));
                end
            end
            if ~isempty(obj.ShadowHandle) && ishandle(obj.ShadowHandle)
                ud = get(obj.ShadowHandle, 'UserData');
                if isstruct(ud) && isfield(ud, 'baseX') && isfield(ud, 'baseY')
                    set(obj.ShadowHandle, ...
                        'XData', ud.baseX + obj.State(1), ...
                        'YData', ud.baseY + obj.State(2));
                end
            end
            obj.updateRunningLights();
        end

        function updateRunningLights(obj)
            if isempty(obj.RunningLightHandles)
                return;
            end
            if ~obj.LightsOn
                for i = 1:length(obj.RunningLightHandles)
                    h = obj.RunningLightHandles{i};
                    if ishandle(h); set(h, 'Visible', 'off'); end
                end
                return;
            end
            vel = norm(obj.State(8:10));
            isGait = false;
            if isprop(obj, 'GaitEnabled') && obj.GaitEnabled
                isGait = true;
            end
            if isGait
                color = [0.9 0.1 0.1];
            elseif vel > 0.1
                color = [0.2 0.8 0.2];
            else
                color = [1.0 0.7 0.1];
            end
            for i = 1:length(obj.RunningLightHandles)
                h = obj.RunningLightHandles{i};
                if ishandle(h)
                    set(h, 'FaceColor', color, 'Visible', 'on');
                end
            end
        end
    end

    methods (Access = private)
        function updatePoseFromState(obj)
            %UPDATEPOSEFROMSTATE  Sync Pose.position/orientation from State.
            obj.Pose.position = obj.State(1:3);
            R = robot.Utils.quatToRotmx(obj.State(4:7));
            [roll, pitch, yaw] = robot.Utils.rotmxToRPY(R);
            obj.Pose.orientation = [roll; pitch; yaw];
        end

        function recordTrailPoint(obj)
            %RECORDTRAILPOINT  Append current position to trail if moving.
            vel = obj.State(8:10);
            if norm(vel) > 0.02
                obj.TrailBuffer(end+1,:) = obj.State(1:3)';
                if size(obj.TrailBuffer, 1) > obj.TrailMaxLen
                    obj.TrailBuffer(1:size(obj.TrailBuffer,1)-obj.TrailMaxLen,:) = [];
                end
            end
        end
    end
end
