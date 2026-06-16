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
        end

        function reset(obj)
            %RESET  Restore state and control to initial conditions.
            obj.State = obj.InitialState;
            obj.Control = obj.InitialControl;
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
    end
end
