classdef TestRobot < robot.Robot
    methods
        function move(obj, direction, amount)
        end
        function [verts, faces, edges] = buildGeometry(obj)
            verts = [0 0 0];
            faces = [];
            edges = [];
        end
        function dstate = computeDynamics(obj, t, state, control)
            dstate = zeros(13, 1);
        end
    end
end
