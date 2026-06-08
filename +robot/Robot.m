classdef Robot < handle
    properties
        Pose                (1,1) struct
        Params              (1,1) struct
        State               (13,1) double
        Control             (:,1) double
        InitialState        (13,1) double
        GraphicsTransform
    end

    methods (Abstract)
        move(obj, direction, amount)
        [verts, faces, edges] = buildGeometry(obj)
        dstate = computeDynamics(obj, t, state, control)
    end

    methods
        function obj = Robot(params)
            arguments
                params (1,1) struct
            end
            obj.Params = params;
            obj.State = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            obj.InitialState = obj.State;
            obj.Control = [];
            obj.Pose = struct('position', [0; 0; 0], 'orientation', [0; 0; 0]);
            obj.updatePoseFromState();
        end

        function step(obj, t, dt)
            arguments
                obj
                t (1,1) double
                dt (1,1) double
            end
            obj.updatePoseFromState();
        end

        function reset(obj)
            obj.State = obj.InitialState;
            obj.Control = [];
            obj.updatePoseFromState();
        end

        function hg = plot(obj, ax)
            hg = hgtransform(ax);
            obj.GraphicsTransform = hg;
        end

        function s = getState(obj)
            s = obj.State;
        end

        function setState(obj, s)
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
            obj.Pose.position = obj.State(1:3);
            R = Utils.quatToRotmx(obj.State(4:7));
            [roll, pitch, yaw] = Utils.rotmxToRPY(R);
            obj.Pose.orientation = [roll; pitch; yaw];
        end
    end
end
