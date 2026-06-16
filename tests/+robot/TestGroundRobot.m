classdef TestGroundRobot < robot.GroundRobot
    methods
        function move(obj, direction, amount)
        end
        function [verts, faces, edges] = buildGeometry(obj)
            verts = [0 0 0];
            faces = [];
            edges = [];
        end
        function dstate = computeDynamics(obj, t, state, control)
            q = state(4:7);
            R = robot.Utils.quatToRotmx(q);
            dstate = [R * state(8:10); zeros(4,1); control(1); 0; 0; 0; 0; 0];
        end
        function n = getControlDimPub(obj)
            n = obj.getControlDim();
        end
    end
end
