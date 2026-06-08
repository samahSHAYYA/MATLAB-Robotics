classdef Quadcopter < robot.AerialRobot

    properties
        armLength  (1,1) double
        bodySize   (1,3) double
        mass       (1,1) double
        inertia    (3,3) double
        maxThrust  (1,1) double
        kTorque    (1,1) double = 0.01
    end

    methods
        function obj = Quadcopter(params)
            arguments
                params (1,1) struct
            end
            obj@robot.AerialRobot(params);
            obj.armLength = params.geometry.armLength;
            obj.bodySize = params.geometry.bodySize;
            obj.mass = params.dynamics.mass;
            obj.inertia = params.dynamics.inertia;
            obj.maxThrust = params.dynamics.maxThrust;
            if isfield(params.dynamics, 'kTorque')
                obj.kTorque = params.dynamics.kTorque;
            end
            obj.Control = zeros(4, 1);
        end

        function move(obj, direction, amount)
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount)) * obj.maxThrust;

            switch direction
                case robot.Direction.UP
                    delta = [1, 1, 1, 1] * amount;
                case robot.Direction.DOWN
                    delta = -[1, 1, 1, 1] * amount;
                case robot.Direction.FORWARD
                    delta = [0, 0, 1, 1] * amount;
                case robot.Direction.BACKWARD
                    delta = [1, 1, 0, 0] * amount;
                case robot.Direction.LEFT
                    delta = [1, 0, 1, 0] * amount;
                case robot.Direction.RIGHT
                    delta = [0, 1, 0, 1] * amount;
                case robot.Direction.YAW_LEFT
                    delta = [-1, 1, 1, -1] * amount;
                case robot.Direction.YAW_RIGHT
                    delta = [1, -1, -1, 1] * amount;
                case robot.Direction.ROLL_LEFT
                    delta = [1, -1, 1, -1] * amount;
                case robot.Direction.ROLL_RIGHT
                    delta = [-1, 1, -1, 1] * amount;
                case robot.Direction.PITCH_UP
                    delta = [-1, -1, 1, 1] * amount;
                case robot.Direction.PITCH_DOWN
                    delta = [1, 1, -1, -1] * amount;
                case robot.Direction.STOP
                    obj.Control = zeros(4, 1);
                    return;
                case robot.Direction.RESET
                    obj.reset();
                    return;
                otherwise
                    return;
            end

            if isempty(obj.Control)
                obj.Control = zeros(4, 1);
            end
            obj.Control = obj.Control + delta';
            obj.Control = max(0, min(obj.maxThrust, obj.Control));
        end

        function [verts, faces, edges] = buildGeometry(obj)
            L = obj.armLength;
            bx = obj.bodySize(1)/2;
            by = obj.bodySize(2)/2;
            bz = obj.bodySize(3)/2;

            bv = [-bx, -by, -bz;  bx, -by, -bz;  bx,  by, -bz; -bx,  by, -bz;
                  -bx, -by,  bz;  bx, -by,  bz;  bx,  by,  bz; -bx,  by,  bz];
            bf = [1, 2, 3, 4;
                  5, 8, 7, 6;
                  1, 5, 6, 2;
                  3, 7, 8, 4;
                  1, 4, 8, 5;
                  2, 6, 7, 3];
            be = [1, 2; 2, 3; 3, 4; 4, 1;
                  5, 6; 6, 7; 7, 8; 8, 5;
                  1, 5; 2, 6; 3, 7; 4, 8];

            r = [ L, -L, 0;  L,  L, 0; -L, -L, 0; -L,  L, 0];

            av = [0, 0, 0; r];

            nBody = size(bv, 1);
            ae = zeros(4, 2);
            for i = 1:4
                ae(i, :) = [nBody + 1, nBody + 1 + i];
            end

            nDisk = 8;
            rr = 0.03;
            th = linspace(0, 2*pi, nDisk+1)';
            th(end) = [];
            circ = [cos(th), sin(th), zeros(nDisk, 1)] * rr;

            dv = zeros(nDisk * 4, 3);
            for i = 1:4
                dv((i-1)*nDisk + 1 : i*nDisk, :) = circ + r(i, :);
            end

            nArm = size(av, 1);
            verts = [bv; av; dv];

            df = NaN(4 * nDisk, 4);
            for i = 1:4
                tipIdx = nBody + 1 + i;
                for j = 1:nDisk
                    j1 = nBody + nArm + (i-1)*nDisk + j;
                    j2 = nBody + nArm + (i-1)*nDisk + mod(j, nDisk) + 1;
                    df((i-1)*nDisk + j, :) = [tipIdx, j1, j2, NaN];
                end
            end

            de = zeros(4 * nDisk * 2, 2);
            for i = 1:4
                tipIdx = nBody + 1 + i;
                for j = 1:nDisk
                    j1 = nBody + nArm + (i-1)*nDisk + j;
                    j2 = nBody + nArm + (i-1)*nDisk + mod(j, nDisk) + 1;
                    de((i-1)*nDisk*2 + j, :) = [j1, j2];
                    de((i-1)*nDisk*2 + nDisk + j, :) = [tipIdx, j1];
                end
            end

            faces = [bf; df];
            edges = [be; ae; de];
        end

        function dstate = computeDynamics(obj, t, state, control)
            q = quaternion(state(4:7)');
            vel = state(8:10);
            omega = state(11:13);
            R = rotmat(q, 'point');

            L = obj.armLength;
            r1 = [ L; -L; 0];
            r2 = [ L;  L; 0];
            r3 = [-L; -L; 0];
            r4 = [-L;  L; 0];

            T = control;

            F_body = [0; 0; sum(T)];

            M1 = cross(r1, [0; 0; T(1)]);
            M2 = cross(r2, [0; 0; T(2)]);
            M3 = cross(r3, [0; 0; T(3)]);
            M4 = cross(r4, [0; 0; T(4)]);

            k = obj.kTorque;
            tau1 = [0; 0; -k * T(1)];
            tau2 = [0; 0;  k * T(2)];
            tau3 = [0; 0;  k * T(3)];
            tau4 = [0; 0; -k * T(4)];

            T_body = M1 + M2 + M3 + M4 + tau1 + tau2 + tau3 + tau4;

            g_world = [0; 0; -9.81];
            g_body = R' * g_world;

            dvel = F_body / obj.mass + g_body - cross(omega, vel);

            I = obj.inertia;
            domega = I \ (T_body - cross(omega, I * omega));

            dpos = R * vel;

            omegaQ = quaternion(0, omega(1), omega(2), omega(3));
            dq = compact(0.5 * q * omegaQ)';

            dstate = [dpos; dq; dvel; domega];
        end

        function hg = plot(obj, ax)
            hg = plot@robot.Robot(obj, ax);
            [verts, faces, edges] = obj.buildGeometry();
            patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
                  'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            for i = 1:size(edges, 1)
                line('Parent', hg, ...
                     'XData', verts(edges(i,:), 1), ...
                     'YData', verts(edges(i,:), 2), ...
                     'ZData', verts(edges(i,:), 3), ...
                     'Color', 'k', 'LineWidth', 1.5);
            end
        end
    end
end
