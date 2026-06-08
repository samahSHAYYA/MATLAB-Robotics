classdef Quadruped < robot.GroundRobot

    properties
        bodyLength    (1,1) double
        bodyWidth     (1,1) double
        bodyHeight    (1,1) double
        legLength1    (1,1) double
        legLength2    (1,1) double
        shoulderWidth (1,1) double
        mass          (1,1) double
        inertia       (3,3) double
        k_contact     (1,1) double
        b_contact     (1,1) double
        mu            (1,1) double = 0.8
    end

    methods
        function obj = Quadruped(params)
            arguments
                params (1,1) struct
            end
            obj@robot.GroundRobot(params);
            obj.bodyLength = params.geometry.bodyLength;
            obj.bodyWidth = params.geometry.bodyWidth;
            obj.bodyHeight = params.geometry.bodyHeight;
            obj.legLength1 = params.geometry.legLength1;
            obj.legLength2 = params.geometry.legLength2;
            obj.shoulderWidth = params.geometry.shoulderWidth;
            obj.mass = params.dynamics.mass;
            obj.inertia = params.dynamics.inertia;
            obj.k_contact = params.dynamics.k_contact;
            obj.b_contact = params.dynamics.b_contact;
            if isfield(params.dynamics, 'mu')
                obj.mu = params.dynamics.mu;
            end
            obj.Control = zeros(6, 1);
            obj.State(3) = obj.bodyHeight/2 + obj.legLength1 + obj.legLength2;
            obj.InitialState = obj.State;
        end

        function move(obj, direction, amount)
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount));
            maxForce = obj.mass * 9.81 * 2;
            maxTorque = mean(diag(obj.inertia)) * 50;
            F = amount * maxForce;
            T = amount * maxTorque;
            switch direction
                case robot.Direction.FORWARD
                    obj.Control = [ F; 0; 0; 0; 0; 0];
                case robot.Direction.BACKWARD
                    obj.Control = [-F; 0; 0; 0; 0; 0];
                case robot.Direction.LEFT
                    obj.Control = [0;  F; 0; 0; 0; 0];
                case robot.Direction.RIGHT
                    obj.Control = [0; -F; 0; 0; 0; 0];
                case robot.Direction.UP
                    obj.Control = [0; 0;  F; 0; 0; 0];
                case robot.Direction.DOWN
                    obj.Control = [0; 0; -F; 0; 0; 0];
                case robot.Direction.YAW_LEFT
                    obj.Control = [0; 0; 0; 0; 0;  T];
                case robot.Direction.YAW_RIGHT
                    obj.Control = [0; 0; 0; 0; 0; -T];
                case robot.Direction.ROLL_LEFT
                    obj.Control = [0; 0; 0;  T; 0; 0];
                case robot.Direction.ROLL_RIGHT
                    obj.Control = [0; 0; 0; -T; 0; 0];
                case robot.Direction.PITCH_UP
                    obj.Control = [0; 0; 0; 0;  T; 0];
                case robot.Direction.PITCH_DOWN
                    obj.Control = [0; 0; 0; 0; -T; 0];
                case robot.Direction.STOP
                    obj.Control = zeros(6, 1);
                case robot.Direction.RESET
                    obj.reset();
                otherwise
                    return;
            end
        end

        function [verts, faces, edges] = buildGeometry(obj)
            bx = obj.bodyLength / 2;
            by = obj.bodyWidth / 2;
            bz = obj.bodyHeight / 2;

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

            sw = obj.shoulderWidth;
            L1 = obj.legLength1;
            L2 = obj.legLength2;

            sFL = [ bx;  sw; 0];
            sFR = [ bx; -sw; 0];
            sHL = [-bx;  sw; 0];
            sHR = [-bx; -sw; 0];

            kv = [sFL'; sFR'; sHL'; sHR'] + [0, 0, -L1];
            fv = [sFL'; sFR'; sHL'; sHR'] + [0, 0, -(L1+L2)];

            % Add shoulder vertices at proper positions
            sv = [sFL'; sFR'; sHL'; sHR'];

            nBody = size(bv, 1);
            nShoulder = size(sv, 1);

            verts = [bv; sv; kv; fv];

            % Leg edges: shoulder -> knee -> foot
            le = [nBody+1, nBody+nShoulder+1;   nBody+nShoulder+1, nBody+nShoulder+nKnee+1;
                  nBody+2, nBody+nShoulder+2;   nBody+nShoulder+2, nBody+nShoulder+nKnee+2;
                  nBody+3, nBody+nShoulder+3;   nBody+nShoulder+3, nBody+nShoulder+nKnee+3;
                  nBody+4, nBody+nShoulder+4;   nBody+nShoulder+4, nBody+nShoulder+nKnee+4];

            edges = [be; le];
            faces = bf;
        end

        function dstate = computeDynamics(obj, t, state, control)
            q = quaternion(state(4:7)');
            vel = state(8:10);
            omega = state(11:13);
            R = rotmat(q, 'point');

            g_world = [0; 0; -9.81];
            g_body = R' * g_world;

            F_control = control(1:3);
            T_control = control(4:6);

            F_contact_body = [0; 0; 0];
            T_contact = [0; 0; 0];

            sFL = [ obj.bodyLength/2;  obj.shoulderWidth; 0];
            sFR = [ obj.bodyLength/2; -obj.shoulderWidth; 0];
            sHL = [-obj.bodyLength/2;  obj.shoulderWidth; 0];
            sHR = [-obj.bodyLength/2; -obj.shoulderWidth; 0];
            shoulders = [sFL, sFR, sHL, sHR];

            for i = 1:4
                foot_body = shoulders(:,i) + [0; 0; -(obj.legLength1 + obj.legLength2)];

                foot_world = R * foot_body + state(1:3);

                foot_vel_world = R * (vel + cross(omega, foot_body));

                if foot_world(3) < 0
                    penetration = -foot_world(3);
                    penetration_vel = -foot_vel_world(3);
                    Fn = obj.k_contact * penetration + obj.b_contact * penetration_vel;
                    Fn = max(0, Fn);

                    v_horiz = foot_vel_world(1:2);
                    v_horiz_norm = norm(v_horiz);
                    if v_horiz_norm > 1e-6
                        Ff = -obj.mu * Fn * (v_horiz / v_horiz_norm);
                    else
                        Ff = [0; 0];
                    end

                    F_contact_world = [Ff(1); Ff(2); Fn];
                    F_contact_body = F_contact_body + R' * F_contact_world;
                    T_contact = T_contact + cross(foot_body, R' * F_contact_world);
                end
            end

            F_total_body = F_control + g_body * obj.mass + F_contact_body;
            dvel = F_total_body / obj.mass - cross(omega, vel);

            I = obj.inertia;
            domega = I \ (T_control + T_contact - cross(omega, I * omega));

            dpos = R * vel;

            omegaQ = quaternion(0, omega(1), omega(2), omega(3));
            dq = compact(0.5 * q * omegaQ)';

            dstate = [dpos; dq; dvel; domega];
        end

        function hg = plot(obj, ax)
            hg = plot@robot.Robot(obj, ax);
            [verts, faces, edges] = obj.buildGeometry();
            patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
                  'FaceColor', [0.7 0.8 0.7], 'EdgeColor', 'none');
            for i = 1:size(edges, 1)
                line('Parent', hg, ...
                     'XData', verts(edges(i,:), 1), ...
                     'YData', verts(edges(i,:), 2), ...
                     'ZData', verts(edges(i,:), 3), ...
                     'Color', 'k', 'LineWidth', 1.5);
            end
        end
    end

    methods (Access = protected)
        function n = getControlDim(obj)
            n = 6;
        end
    end
end
