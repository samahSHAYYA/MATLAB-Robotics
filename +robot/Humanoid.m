classdef Humanoid < robot.GroundRobot
    properties
        bodyHeight    (1,1) double
        bodyWidth     (1,1) double
        hipWidth      (1,1) double
        thighLength   (1,1) double
        shinLength    (1,1) double
        footLength    (1,1) double
        footWidth     (1,1) double
        footThickness (1,1) double
        mass          (1,1) double
        inertia       (3,3) double
        k_contact     (1,1) double
        b_contact     (1,1) double
        mu            (1,1) double = 0.9
        FootPositions    (2,3) double
        KneePositions    (2,3) double
        FootBoxVerts     (2,8,3) double
        ArmGraphics      (1,2) cell
        ArmLines
        ArmJoints
        LegLines
        LegJoints
        TorsoPatch
        HeadSurf
        JointAngles      (2,3) double
        LegGraphics      (1,2) cell
        GaitPhase        (2,1) double = [0; 0.5]
        GaitEnabled      (1,1) logical = false
        StepHeight       (1,1) double = 0.015
        StepLength       (1,1) double = 0.06
        GaitFrequency    (1,1) double = 0.5
        LateralShift     (1,1) double = 0
        FrontIndicator
        BalanceGainP     (1,1) double = 1000
        BalanceGainD     (1,1) double = 120
    end

    methods
        function obj = Humanoid(params)
            arguments
                params (1,1) struct
            end
            obj@robot.GroundRobot();
            obj.bodyHeight = params.geometric.bodyHeight;
            obj.bodyWidth = params.geometric.bodyWidth;
            obj.hipWidth = params.geometric.hipWidth;
            obj.thighLength = params.kinematic.thighLength;
            obj.shinLength = params.kinematic.shinLength;
            obj.footLength = params.kinematic.footLength;
            obj.footWidth = obj.footLength * 0.7;
            obj.footThickness = 0.03;
            obj.mass = params.dynamic.mass;
            obj.inertia = params.dynamic.inertia;
            obj.k_contact = params.elastic.k_contact;
            obj.b_contact = params.elastic.b_contact;
            if isfield(params.elastic, 'mu')
                obj.mu = params.elastic.mu;
            end
            if isfield(params, 'balance')
                if isfield(params.balance, 'gainP')
                    obj.BalanceGainP = params.balance.gainP;
                end
                if isfield(params.balance, 'gainD')
                    obj.BalanceGainD = params.balance.gainD;
                end
            end
            validateattributes(obj.thighLength, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.shinLength, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.footLength, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.mass, {'double'}, {'scalar', 'positive'});
            obj.Control = zeros(6, 1);
            obj.InitialControl = obj.Control;
            obj.State(3) = obj.thighLength + obj.shinLength;
            obj.InitialState = obj.State;

            hw = obj.hipWidth;
            L1 = obj.thighLength;
            L2 = obj.shinLength;
            hipPos = [-hw, 0, 0; hw, 0, 0];
            obj.FootPositions = hipPos + [0, 0, -(L1 + L2)];
            obj.KneePositions = hipPos + [0, 0, -L1];
            obj.JointAngles = zeros(2, 3);
            obj.LegGraphics = cell(1, 2);
            obj.ArmGraphics = cell(1, 2);
            obj.initFootBoxVerts();
        end

        function step(obj, t, dt)
            if obj.GaitEnabled
                phaseInc = obj.GaitFrequency * dt;
                obj.GaitPhase = obj.GaitPhase + phaseInc;
                obj.GaitPhase(obj.GaitPhase >= 1) = ...
                    obj.GaitPhase(obj.GaitPhase >= 1) - 1;
            end
            obj.updateLegPositions();
            step@robot.GroundRobot(obj, t, dt);
            obj.updateWireframe();
        end

        function move(obj, direction, amount)
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount));
            maxForce = obj.mass * 9.81 * 0.02;
            maxTorque = mean(diag(obj.inertia)) * 5;
            F = amount * maxForce;
            T = amount * maxTorque;
            switch direction
                case robot.Direction.FORWARD
                    obj.Control = [0;  F; 0; 0; 0; 0];
                case robot.Direction.BACKWARD
                    obj.Control = [0; -F; 0; 0; 0; 0];
                case robot.Direction.LEFT
                    obj.Control = [0; 0; 0; 0; 0;  T];
                case robot.Direction.RIGHT
                    obj.Control = [0; 0; 0; 0; 0; -T];
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
                    obj.State(8:13) = 0;
                case robot.Direction.RESET
                    obj.reset();
                otherwise
                    return;
            end
        end

        function toggleGait(obj)
            obj.GaitEnabled = ~obj.GaitEnabled;
            if ~obj.GaitEnabled
                obj.GaitPhase = [0; 0.5];
            end
        end

        function reset(obj)
            reset@robot.Robot(obj);
            obj.GaitPhase = [0; 0.5];
            obj.GaitEnabled = false;
        end

        function [verts, faces, edges] = buildGeometry(obj)
            bw2 = obj.bodyWidth / 2;
            hw4 = obj.hipWidth / 4;
            bh = obj.bodyHeight;
            cz = bh * 0.4;
            hz = bh / 2;

            verts = [-bw2, -hw4, cz-hz;  bw2, -hw4, cz-hz;  bw2,  hw4, cz-hz; -bw2,  hw4, cz-hz;
                     -bw2, -hw4, cz+hz;  bw2, -hw4, cz+hz;  bw2,  hw4, cz+hz; -bw2,  hw4, cz+hz];
            faces = [1,2,3,4; 5,8,7,6; 1,5,6,2; 3,7,8,4; 1,4,8,5; 2,6,7,3];
            edges = [1,2; 2,3; 3,4; 4,1; 5,6; 6,7; 7,8; 8,5; 1,5; 2,6; 3,7; 4,8];
        end

        function dstate = computeDynamics(obj, ~, state, control)
            q = state(4:7);
            vel = state(8:10);
            omega = state(11:13);
            R = robot.Utils.quatToRotmx(q);

            g_world = [0; 0; -9.81];
            g_body = R' * g_world;

            F_control = control(1:3);
            T_control = control(4:6);

            roll_angle = atan2(-g_body(2), -g_body(3));
            pitch_angle = atan2(g_body(1), -g_body(3));
            if abs(control(6)) < 1e-9
                yawDamp = obj.BalanceGainD;
            else
                yawDamp = obj.BalanceGainD * 0.01;
            end
            T_balance = ...
                [-obj.BalanceGainD * omega(1) - obj.BalanceGainP * roll_angle;
                 -obj.BalanceGainD * omega(2) - obj.BalanceGainP * pitch_angle;
                 -yawDamp * omega(3)];
            T_control = T_control + T_balance;

            F_contact_body = [0; 0; 0];
            T_contact = [0; 0; 0];

            fw2 = obj.footWidth / 2;
            fl2 = obj.footLength / 2;
            ft = obj.footThickness;
            footCorners = [-fw2, -fl2, -ft; fw2, -fl2, -ft; fw2, fl2, -ft; -fw2, fl2, -ft];

            fwdCmd = control(2) / (obj.mass * 9.81 * 0.02 + eps);
            fwdCmd = max(-1, min(1, fwdCmd));

            for i = 1:2
                ankle_body = obj.FootPositions(i, :)';
                phase_i = obj.GaitPhase(i);

                if phase_i >= 0.5
                    v_foot_body = [0; -obj.StepLength * fwdCmd * 2 * obj.GaitFrequency; 0];
                else
                    v_foot_body = [0; 0; 0];
                end

                for c = 1:4
                    contact_body = ankle_body + footCorners(c, :)';
                    contact_world = R * contact_body + state(1:3);
                    contact_vel_world = R * (vel + cross(omega, contact_body) + v_foot_body);

                    if contact_world(3) < 0
                        penetration = -contact_world(3);
                        penetration_vel = -contact_vel_world(3);
                        Fn = obj.k_contact * penetration + obj.b_contact * penetration_vel;
                        Fn = max(0, Fn);

                        v_horiz = contact_vel_world(1:2);
                        v_horiz_norm = norm(v_horiz);
                        if v_horiz_norm > 1e-6
                            Ff = -obj.mu * Fn * (v_horiz / v_horiz_norm);
                        else
                            Ff = [0; 0];
                        end

                        F_contact_world = [Ff(1); Ff(2); Fn];
                        F_contact_body = F_contact_body + R' * F_contact_world;
                        T_contact = T_contact + cross(contact_body, R' * F_contact_world);
                    end
                end
            end

            F_total_body = F_control + g_body * obj.mass + F_contact_body;
            dvel = F_total_body / obj.mass - cross(omega, vel);

            I = obj.inertia;
            domega = I \ (T_control + T_contact - cross(omega, I * omega));

            dpos = R * vel;

            omegaQ = [0; omega(1); omega(2); omega(3)];
            dq = 0.5 * robot.Utils.quatMultiply(q, omegaQ);

            dstate = [dpos; dq; dvel; domega];
            if any(~isfinite(dstate))
                dstate(:) = 0;
                warning('Non-finite dynamics detected — zeroing output.');
            end
        end

        function hg = plot(obj, ax)
            hg = plot@robot.Robot(obj, ax);
            hold(ax, 'on');

            [bv, bf, ~] = obj.buildGeometry();
            obj.TorsoPatch = patch('Parent', hg, 'Vertices', bv, 'Faces', bf, ...
                'FaceColor', [0.61 0.35 0.71], ...
                'EdgeColor', [0.41 0.20 0.51], 'LineWidth', 1.5);

            [X, Y, Z] = sphere(12);
            headR = obj.bodyWidth * 0.2;
            hz2 = obj.bodyHeight * 0.85;
            obj.HeadSurf = surf(X*headR, Y*headR, Z*headR + hz2, ...
                'Parent', hg, 'FaceColor', [0.85 0.75 0.90], 'EdgeColor', 'none');

            obj.ArmGraphics = cell(1, 2);
            obj.ArmLines = gobjects(2, 2);
            obj.ArmJoints = gobjects(2, 3);
            bh = obj.bodyHeight;
            for i = 1:2
                armTf = hgtransform(hg);
                obj.ArmGraphics{i} = armTf;
                side = 2*(i-1) - 1;
                sx = side * obj.bodyWidth * 0.55;
                sh = [sx, 0, bh*0.65];
                el = [sx, 0, bh*0.40];
                ha = [sx, 0, bh*0.15];
                obj.ArmLines(i,1) = line('Parent', armTf, ...
                    'XData', [sh(1), el(1)], 'YData', [sh(2), el(2)], 'ZData', [sh(3), el(3)], ...
                    'Color', [0.75 0.50 0.80], 'LineWidth', 3);
                obj.ArmLines(i,2) = line('Parent', armTf, ...
                    'XData', [el(1), ha(1)], 'YData', [el(2), ha(2)], 'ZData', [el(3), ha(3)], ...
                    'Color', [0.65 0.40 0.70], 'LineWidth', 2.5);
                [cx, cy, cz] = sphere(8);
                joints = [sh; el; ha];
                for j = 1:3
                    obj.ArmJoints(i,j) = surf(cx*0.02 + joints(j,1), cy*0.02 + joints(j,2), ...
                        cz*0.02 + joints(j,3), 'Parent', armTf, ...
                        'FaceColor', [0.75 0.50 0.80], 'EdgeColor', 'none');
                end
            end

            hw = obj.hipWidth;
            L1 = obj.thighLength;
            L2 = obj.shinLength;
            obj.LegGraphics = cell(1, 2);
            obj.LegLines = gobjects(2, 2);
            obj.LegJoints = gobjects(2, 3);
            for i = 1:2
                legTf = hgtransform(hg);
                obj.LegGraphics{i} = legTf;
                side = 2*(i-1) - 1;
                hp = [side*hw, 0, 0];
                kp = [side*hw, 0, -L1];
                fp = [side*hw, 0, -(L1+L2)];
                obj.LegLines(i,1) = line('Parent', legTf, ...
                    'XData', [hp(1), kp(1)], 'YData', [hp(2), kp(2)], 'ZData', [hp(3), kp(3)], ...
                    'Color', [0.30 0.70 0.40], 'LineWidth', 3);
                obj.LegLines(i,2) = line('Parent', legTf, ...
                    'XData', [kp(1), fp(1)], 'YData', [kp(2), fp(2)], 'ZData', [kp(3), fp(3)], ...
                    'Color', [0.25 0.60 0.35], 'LineWidth', 3);
                [cx, cy, cz] = sphere(8);
                joints = [hp; kp; fp];
                for j = 1:3
                    obj.LegJoints(i,j) = surf(cx*0.025 + joints(j,1), cy*0.025 + joints(j,2), ...
                        cz*0.025 + joints(j,3), 'Parent', legTf, ...
                        'FaceColor', [0.25 0.60 0.35], 'EdgeColor', 'none');
                end
            end

            fd = obj.bodyWidth / 2;
            nose = [0, fd+0.03, 0; -0.04, fd+0.01, -0.04; 0.04, fd+0.01, -0.04; 0, fd+0.01, 0.04];
            obj.FrontIndicator = patch('Parent', hg, ...
                'Vertices', nose, 'Faces', [1,2,3; 1,3,4; 1,4,2; 2,4,3], ...
                'FaceColor', [0.9 0.1 0.1], 'EdgeColor', 'k');
            line('Parent', hg, ...
                'XData', [0, 0], 'YData', [fd+0.03, fd+0.06], 'ZData', [0, 0], ...
                'Color', [0.9 0.1 0.1], 'LineWidth', 2);
        end
    end

    methods (Access = protected)
        function n = getControlDim(~)
            n = 6;
        end
    end

    methods (Access = private)
        function [theta1, theta2, theta3] = legIK(obj, footPos, hipPos)
            r = footPos - hipPos;
            x = r(1); y = r(2); z = r(3);
            L1 = obj.thighLength;
            L2 = obj.shinLength;

            theta1 = atan2(y, x);

            d = sqrt(x^2 + y^2 + z^2);
            d = max(d, abs(L1 - L2) + 1e-12);
            d = min(d, L1 + L2 - 1e-12);

            cos_theta3 = (d^2 - L1^2 - L2^2) / (2 * L1 * L2);
            cos_theta3 = max(-1, min(1, cos_theta3));
            theta3 = acos(cos_theta3);

            theta2 = atan2(sqrt(x^2 + y^2), -z) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
        end

        function footPos = legFK(obj, theta1, theta2, theta3, hipPos)
            L1 = obj.thighLength;
            L2 = obj.shinLength;

            knee_local = [L1 * sin(theta2); 0; -L1 * cos(theta2)];
            foot_dir = [sin(theta2 + theta3); 0; -cos(theta2 + theta3)];

            Rz = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
            knee = Rz * knee_local;
            foot_local = Rz * (L2 * foot_dir);

            footPos = hipPos(:)' + knee(:)' + foot_local(:)';
        end

        function updateLegPositions(obj)
            hw = obj.hipWidth;
            L1 = obj.thighLength;
            L2 = obj.shinLength;
            hipPos = [-hw, 0, 0; hw, 0, 0];

            for i = 1:2
                if obj.GaitEnabled
                    foot_target = obj.computeFootTarget(i);
                else
                    foot_target = hipPos(i, :) + [0, 0, -(L1 + L2)];
                end

                [t1, t2, t3] = obj.legIK(foot_target, hipPos(i, :));
                obj.JointAngles(i, :) = [t1, t2, t3];

                knee = [L1 * sin(t2), 0, -L1 * cos(t2)];
                Rz = [cos(t1), -sin(t1), 0; sin(t1), cos(t1), 0; 0, 0, 1];
                knee = (Rz * knee')';
                obj.KneePositions(i, :) = hipPos(i, :) + knee;
                obj.FootPositions(i, :) = obj.legFK(t1, t2, t3, hipPos(i, :));
            end
            obj.initFootBoxVerts();
        end

        function footTarget = computeFootTarget(obj, legIdx)
            phase = obj.GaitPhase(legIdx);
            hip = obj.getHipPos(legIdx);
            defaultFoot = hip + [0, 0, -(obj.thighLength + obj.shinLength)];

            fwdCmd = obj.Control(2) / (obj.mass * 9.81 * 0.02 + eps);
            fwdCmd = max(-1, min(1, fwdCmd));
            stepDy = obj.StepLength * fwdCmd;
            stepDz = obj.StepHeight * (0.3 + 0.7 * abs(fwdCmd));

            sway = obj.LateralShift * sin(2 * pi * obj.GaitPhase(legIdx));

            if phase < 0.5
                t = phase / 0.5;
                dy = stepDy * (t - 0.5);
                dz = stepDz * sin(pi * t);
            else
                t = (phase - 0.5) / 0.5;
                dy = stepDy * (0.5 - t);
                dz = 0;
            end

            footTarget = defaultFoot + [sway, dy, dz];
        end

        function pos = getHipPos(obj, legIdx)
            hw = obj.hipWidth;
            tbl = [-hw, 0, 0; hw, 0, 0];
            pos = tbl(legIdx, :);
        end

        function initFootBoxVerts(obj)
            fw2 = obj.footWidth / 2;
            fl2 = obj.footLength / 2;
            ft = obj.footThickness;
            for i = 1:2
                a = obj.FootPositions(i, :);
                obj.FootBoxVerts(i, :, :) = a + [-fw2, -fl2, -ft; fw2, -fl2, -ft; fw2, fl2, -ft; -fw2, fl2, -ft;
                                                  -fw2, -fl2,   0; fw2, -fl2,   0; fw2, fl2,   0; -fw2, fl2,   0];
            end
        end

        function updateWireframe(obj)
            if isempty(obj.LegGraphics) || isempty(obj.LegGraphics{1}) || ~isvalid(obj.LegGraphics{1})
                return;
            end

            hw = obj.hipWidth;
            hipPos = [-hw, 0, 0; hw, 0, 0];

            for i = 1:2
                hip = hipPos(i, :);
                knee = obj.KneePositions(i, :);
                foot = obj.FootPositions(i, :);

                set(obj.LegLines(i, 1), ...
                    'XData', [hip(1), knee(1)], ...
                    'YData', [hip(2), knee(2)], ...
                    'ZData', [hip(3), knee(3)]);

                set(obj.LegLines(i, 2), ...
                    'XData', [knee(1), foot(1)], ...
                    'YData', [knee(2), foot(2)], ...
                    'ZData', [knee(3), foot(3)]);

                [cx, cy, cz] = sphere(8);
                r = 0.025;
                set(obj.LegJoints(i, 1), ...
                    'XData', cx*r + hip(1), ...
                    'YData', cy*r + hip(2), ...
                    'ZData', cz*r + hip(3));
                set(obj.LegJoints(i, 2), ...
                    'XData', cx*r + knee(1), ...
                    'YData', cy*r + knee(2), ...
                    'ZData', cz*r + knee(3));
                set(obj.LegJoints(i, 3), ...
                    'XData', cx*r + foot(1), ...
                    'YData', cy*r + foot(2), ...
                    'ZData', cz*r + foot(3));
            end
        end
    end
end
