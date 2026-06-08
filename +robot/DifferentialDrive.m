classdef DifferentialDrive < robot.Robot
    properties
        wheelRadius (1,1) double
        trackWidth  (1,1) double
        mass        (1,1) double
        inertia     (1,1) double
        maxTorque   (1,1) double
    end

    methods
        function obj = DifferentialDrive(params)
            arguments
                params (1,1) struct
            end
            obj@robot.Robot(params);
            obj.wheelRadius = params.geometry.wheelRadius;
            obj.trackWidth = params.geometry.trackWidth;
            obj.mass = params.dynamics.mass;
            obj.inertia = params.dynamics.inertia;
            obj.maxTorque = params.dynamics.maxTorque;
        end

        function move(obj, direction, amount)
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount)) * obj.maxTorque;
            switch direction
                case robot.Direction.FORWARD
                    obj.Control = [ amount;  amount];
                case robot.Direction.BACKWARD
                    obj.Control = [-amount; -amount];
                case {robot.Direction.LEFT, robot.Direction.YAW_LEFT}
                    obj.Control = [-amount;  amount];
                case {robot.Direction.RIGHT, robot.Direction.YAW_RIGHT}
                    obj.Control = [ amount; -amount];
                case robot.Direction.STOP
                    obj.Control = [0; 0];
                case robot.Direction.RESET
                    obj.reset();
                otherwise
            end
        end

        function [verts, faces, edges] = buildGeometry(obj)
            bx = 0.2; by = 0.15; bz = 0.05;
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

            wx = 0.025; wy = 0.1; wz = 0.1;

            lcx = 0; lcy = -obj.trackWidth/2; lcz = 0;
            lv = [lcx-wx, lcy-wy, lcz-wz;  lcx+wx, lcy-wy, lcz-wz;
                  lcx+wx, lcy+wy, lcz-wz;  lcx-wx, lcy+wy, lcz-wz;
                  lcx-wx, lcy-wy, lcz+wz;  lcx+wx, lcy-wy, lcz+wz;
                  lcx+wx, lcy+wy, lcz+wz;  lcx-wx, lcy+wy, lcz+wz];

            rcx = 0; rcy = obj.trackWidth/2; rcz = 0;
            rv = [rcx-wx, rcy-wy, rcz-wz;  rcx+wx, rcy-wy, rcz-wz;
                  rcx+wx, rcy+wy, rcz-wz;  rcx-wx, rcy+wy, rcz-wz;
                  rcx-wx, rcy-wy, rcz+wz;  rcx+wx, rcy-wy, rcz+wz;
                  rcx+wx, rcy+wy, rcz+wz;  rcx-wx, rcy+wy, rcz+wz];

            wf = [1, 2, 3, 4;
                  5, 8, 7, 6;
                  1, 5, 6, 2;
                  3, 7, 8, 4;
                  1, 4, 8, 5;
                  2, 6, 7, 3];

            we = [1, 2; 2, 3; 3, 4; 4, 1;
                  5, 6; 6, 7; 7, 8; 8, 5;
                  1, 5; 2, 6; 3, 7; 4, 8];

            nBody = size(bv, 1);
            nLeft = size(lv, 1);

            verts = [bv; lv; rv];
            faces = [bf; wf + nBody; wf + nBody + nLeft];
            edges = [be; we + nBody; we + nBody + nLeft];
        end

        function dstate = computeDynamics(obj, t, state, control)
            vx = state(8);
            wz = state(13);
            q = state(4:7);

            R = robot.Utils.quatToRotmx(q);

            F_drive = (control(1) + control(2)) / obj.wheelRadius;
            T_yaw = (control(2) - control(1)) * obj.trackWidth / (2 * obj.wheelRadius);

            dvel_x = F_drive / obj.mass;
            dvel_y = 0;
            dvel_z = 0;

            domega_x = 0;
            domega_y = 0;
            domega_z = T_yaw / obj.inertia;

            dpos = R * [vx; 0; 0];

            omega_Q = [0; 0; 0; wz];
            dq = 0.5 * robot.Utils.quatMultiply(q, omega_Q);

            dstate = [dpos; dq; dvel_x; dvel_y; dvel_z; domega_x; domega_y; domega_z];
        end

        function step(obj, t, dt)
            arguments
                obj
                t (1,1) double
                dt (1,1) double
            end
            if isempty(obj.Control)
                u = [0; 0];
            else
                u = obj.Control;
            end
            dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
            s = DynamicsEngine.rk4Step(dynFun, t, obj.State, u, dt);
            obj.setState(s);
        end

        function hg = plot(obj, ax)
            hg = plot@robot.Robot(obj, ax);
            [verts, faces, edges] = obj.buildGeometry();
            patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
                  'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
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
