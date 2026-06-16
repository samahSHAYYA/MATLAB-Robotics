classdef Quadcopter < robot.AerialRobot
    %QUADCOPTER  6-DOF quadcopter with PD attitude control.
    %   Motor layout (x-forward, y-right):
    %     M1 (FR):  [ L, -L, 0]   M2 (FL): [ L,  L, 0]
    %     M3 (RR):  [-L, -L, 0]   M4 (RL): [-L,  L, 0]
    %   Positive yaw: counter-clockwise from above (M2,M3 ↑, M1,M4 ↓).

    properties
        armLength    (1,1) double
        bodySize     (1,3) double
        mass         (1,1) double
        inertia      (3,3) double
        maxThrust    (1,1) double
        kTorque      (1,1) double = 0.05
        RotorGraphics (1,4) cell
        FrontIndicator
    end

    properties (Access = private)
        DesiredPitch         (1,1) double = 0
        DesiredRoll          (1,1) double = 0
        DesiredYawRate       (1,1) double = 0
        DesiredVerticalThrust (1,1) double = 0
        kpAngle (1,1) double = 2.0
        kdAngle (1,1) double = 0.3
        kpYaw   (1,1) double = 0.2
    end

    methods
        function obj = Quadcopter(params)
            %QUADCOPTER  Construct from parameter struct.
            %   Fields: params.geometric.{armLength, bodySize}
            %           params.dynamic.{mass, inertia, maxThrust, kTorque (opt)}
            arguments
                params (1,1) struct
            end
            obj@robot.AerialRobot();
            obj.armLength = params.geometric.armLength;
            obj.bodySize = params.geometric.bodySize;
            obj.mass = params.dynamic.mass;
            obj.inertia = params.dynamic.inertia;
            obj.maxThrust = params.dynamic.maxThrust;
            if isfield(params.dynamic, 'kTorque')
                obj.kTorque = params.dynamic.kTorque;
            end
            obj.State(3) = 0.5;
            obj.InitialState = obj.State;
            obj.Control = ones(4, 1) * (obj.mass * 9.81 / 4);
            obj.DesiredVerticalThrust = obj.mass * 9.81;
        end

        function move(obj, direction, amount)
            %MOVE  Set desired pitch/roll/yaw/vertical-thrust targets.
            %   move(direction, amount) translates Direction enum to
            %   attitude setpoints. STOP zeros all setpoints → hover.
            %   Inputs: direction - robot.Direction enum
            %           amount    - [0,1] authority scale
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount));
            hover = obj.mass * 9.81;

            switch direction
                case robot.Direction.UP
                    obj.DesiredVerticalThrust = hover * (1 + amount * 0.5);
                case robot.Direction.DOWN
                    obj.DesiredVerticalThrust = hover * (1 - amount * 0.5);
                case robot.Direction.FORWARD
                    obj.DesiredPitch = amount * 0.15;
                    obj.DesiredRoll = 0;
                    obj.DesiredYawRate = 0;
                case robot.Direction.BACKWARD
                    obj.DesiredPitch = -amount * 0.15;
                    obj.DesiredRoll = 0;
                    obj.DesiredYawRate = 0;
                case robot.Direction.LEFT
                    obj.DesiredRoll = -amount * 0.15;
                    obj.DesiredPitch = 0;
                    obj.DesiredYawRate = 0;
                case robot.Direction.RIGHT
                    obj.DesiredRoll = amount * 0.15;
                    obj.DesiredPitch = 0;
                    obj.DesiredYawRate = 0;
                case {robot.Direction.YAW_LEFT, robot.Direction.ROLL_LEFT}
                    obj.DesiredYawRate = amount * 1.0;
                case {robot.Direction.YAW_RIGHT, robot.Direction.ROLL_RIGHT}
                    obj.DesiredYawRate = -amount * 1.0;
                case robot.Direction.PITCH_UP
                    obj.DesiredPitch = -amount * 0.15;
                    obj.DesiredRoll = 0;
                    obj.DesiredYawRate = 0;
                case robot.Direction.PITCH_DOWN
                    obj.DesiredPitch = amount * 0.15;
                    obj.DesiredRoll = 0;
                    obj.DesiredYawRate = 0;
                case robot.Direction.STOP
                    obj.DesiredPitch = 0;
                    obj.DesiredRoll = 0;
                    obj.DesiredYawRate = 0;
                    obj.DesiredVerticalThrust = hover;
                    obj.State(8:13) = 0;
                case robot.Direction.RESET
                    obj.reset();
                    return;
                otherwise
                    return;
            end
        end

        function step(obj, t, dt)
            %STEP  Compute attitude-control thrusts, then RK4-integrate.
            %   Runs computeAttitudeThrusts() each step to track the
            %   desired attitude setpoints via PD control.
            obj.Control = obj.computeAttitudeThrusts();
            step@robot.AerialRobot(obj, t, dt);
        end

        function T = computeAttitudeThrusts(obj)
            %COMPUTEATTITUDETHRUSTS  PD attitude → motor thrust conversion.
            %   Reads current pitch/roll/yaw-rate from state, compares to
            %   DesiredPitch/DesiredRoll/DesiredYawRate, computes torques
            %   with PD law, and distributes as differential thrust.
            %   Outputs: T - 4×1 motor thrusts [0, maxThrust]
            q = quaternion(obj.State(4:7)');
            omega = obj.State(11:13);
            R = rotmat(q, 'point');

            pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
            roll  = atan2(R(3,2), R(3,3));

            L = obj.armLength;
            tau_pitch = obj.kpAngle * (obj.DesiredPitch - pitch) - obj.kdAngle * omega(2);
            tau_roll  = obj.kpAngle * (obj.DesiredRoll - roll)  - obj.kdAngle * omega(1);
            tau_yaw   = obj.kpYaw * (obj.DesiredYawRate - omega(3));

            dp = tau_pitch / (4 * L);
            dr = tau_roll / (4 * L);
            dy = tau_yaw / (4 * obj.kTorque);

            hover = max(0, obj.DesiredVerticalThrust / 4);
            T = hover + [ -dp - dr - dy;
                          -dp + dr + dy;
                          +dp - dr + dy;
                          +dp + dr - dy ];
            T = max(0, min(obj.maxThrust, T));
        end

        function [verts, faces, edges] = buildGeometry(obj)
            %BUILDGEOMETRY  Wireframe vertices/faces/edges for quadcopter.
            %   Body: rectangular box at origin.
            %   Arms: lines from center to 4 motor positions.
            %   Rotors: disks at each motor position.
            %   Returns arrays suitable for patch() and line().
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

        function dstate = computeDynamics(obj, ~, state, control)
            %COMPUTEDYNAMICS  Rigid-body dynamics with motor forces and
            %                 aerodynamic damping plus ground contact.
            %   State: [pos; quat; vel; omega]
            %   Control: 4×1 motor thrusts [T1; T2; T3; T4]
            %   Includes: gravity, rotor forces/torques, drag torque
            %   damping (-0.005*omega), velocity drag (-0.5*vel),
            %   and ground spring-damper at z<0.02.
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

            I = obj.inertia;

            T_damp = -0.005 * omega;
            domega = I \ (T_body + T_damp - cross(omega, I * omega));

            F_drag = -0.5 * vel;
            dvel = F_body / obj.mass + g_body + F_drag / obj.mass - cross(omega, vel);

            if state(3) < 0.02
                penetration = state(3) - 0.02;
                world_vel = R * vel;
                Fz = -20000 * penetration - 200 * min(world_vel(3), 0);
                dvel = dvel + (R' * [0;0;max(0,Fz)]) / obj.mass;
            end

            dpos = R * vel;

            omegaQ = quaternion(0, omega(1), omega(2), omega(3));
            dq = compact(0.5 * q * omegaQ)';

            dstate = [dpos; dq; dvel; domega];
        end

        function reset(obj)
            %RESET  Restore initial state and re-arm hover thrust.
            reset@robot.Robot(obj);
            obj.Control = ones(4, 1) * (obj.mass * 9.81 / 4);
            obj.DesiredPitch = 0;
            obj.DesiredRoll = 0;
            obj.DesiredYawRate = 0;
            obj.DesiredVerticalThrust = obj.mass * 9.81;
        end

        function hg = plot(obj, ax)
            %PLOT  Build full quadcopter visual: body, arms, rotors, nose.
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

            r = [obj.armLength, -obj.armLength, 0;
                 obj.armLength,  obj.armLength, 0;
                -obj.armLength, -obj.armLength, 0;
                -obj.armLength,  obj.armLength, 0];
            [cx, cy, cz] = cylinder(0.015, 8);
            cz = cz * 0.03 - 0.015;
            for i = 1:4
                obj.RotorGraphics{i} = surf(cx + r(i,1), cy + r(i,2), cz + r(i,3), ...
                    'Parent', hg, ...
                    'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
            end

            bx = obj.bodySize(1)/2;
            bz = obj.bodySize(3)/2;
            nose = [bx+0.03, 0, bz;
                    bx, -bz/3, bz;
                    bx, bz/3, bz;
                    bx, 0, bz+0.03];
            obj.FrontIndicator = patch('Parent', hg, ...
                'Vertices', nose, 'Faces', [1,2,3; 1,3,4; 1,4,2; 2,4,3], ...
                'FaceColor', [0.9 0.1 0.1], 'EdgeColor', 'k');

            line('Parent', hg, ...
                'XData', [bx+0.03, bx+0.06], ...
                'YData', [0, 0], ...
                'ZData', [bz, bz], ...
                'Color', [0.9 0.1 0.1], 'LineWidth', 2);
        end
    end
end
