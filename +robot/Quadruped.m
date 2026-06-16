classdef Quadruped < robot.GroundRobot
    %QUADRUPED  6-DOF quadruped with spring-damper foot contact and trot
    %           gait.
    %   Leg numbering: 1=FL (front-left), 2=FR, 3=RL, 4=RR.
    %   Trot gait: diagonal pairs (1+4, 2+3) swing together.
    %   Control: 6-axis wrench [Fx, Fy, Fz, Tx, Ty, Tz] in body frame.

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
        FootPositions    (4,3) double
        KneePositions    (4,3) double
        JointAngles      (4,3) double
        LegGraphics      (1,4) cell
        GaitPhase        (4,1) double = [0; 0.5; 0.5; 0]
        GaitEnabled      (1,1) logical = false
        StepHeight       (1,1) double = 0.04
        StepLength       (1,1) double = 0.03
        GaitFrequency    (1,1) double = 2.0
        GaitTimer        (1,1) double = 0
        FrontIndicator
    end

    methods
        function obj = Quadruped(params)
            %QUADRUPED  Construct from parameter struct.
            %   Fields: params.geometric.{bodyLength, bodyWidth, bodyHeight,
            %                             shoulderWidth}
            %           params.kinematic.{legLength1, legLength2}
            %           params.dynamic.{mass, inertia}
            %           params.elastic.{k_contact, b_contact, mu (opt)}
            arguments
                params (1,1) struct
            end
            obj@robot.GroundRobot();
            obj.bodyLength = params.geometric.bodyLength;
            obj.bodyWidth = params.geometric.bodyWidth;
            obj.bodyHeight = params.geometric.bodyHeight;
            obj.legLength1 = params.kinematic.legLength1;
            obj.legLength2 = params.kinematic.legLength2;
            obj.shoulderWidth = params.geometric.shoulderWidth;
            obj.mass = params.dynamic.mass;
            obj.inertia = params.dynamic.inertia;
            obj.k_contact = params.elastic.k_contact;
            obj.b_contact = params.elastic.b_contact;
            if isfield(params.elastic, 'mu')
                obj.mu = params.elastic.mu;
            end
            validateattributes(obj.bodyLength, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.bodyWidth, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.legLength1, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.legLength2, {'double'}, {'scalar', 'positive'});
            validateattributes(obj.mass, {'double'}, {'scalar', 'positive'});
            obj.Control = zeros(6, 1);
            obj.InitialControl = obj.Control;
            obj.State(3) = obj.bodyHeight/2 + obj.legLength1 + obj.legLength2;
            obj.InitialState = obj.State;

            sw = obj.shoulderWidth;
            bx = obj.bodyLength/2;
            L1 = obj.legLength1;
            L2 = obj.legLength2;
            shoulderPos = [bx,  sw, 0; bx, -sw, 0; -bx,  sw, 0; -bx, -sw, 0];
            obj.FootPositions = shoulderPos + [0, 0, -(L1 + L2)];
            obj.KneePositions = shoulderPos + [0, 0, -L1];
            obj.JointAngles = zeros(4, 3);
            obj.LegGraphics = cell(1, 4);
        end

        function step(obj, t, dt)
            %STEP  Advance gait phase, solve IK, integrate physics,
            %       then update the wireframe leg lines.
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
            %MOVE  Set 6-axis body wrench from direction command.
            %   Control vector: [Fx, Fy, Fz, Tx, Ty, Tz] in body frame.
            %   Translational force ~15% body weight, torque scaled by
            %   mean inertia × 5.
            arguments
                obj
                direction robot.Direction
                amount (1,1) double = 1.0
            end
            amount = max(0, min(1, amount));
            maxForce = obj.mass * 9.81 * 0.15;
            maxTorque = mean(diag(obj.inertia)) * 5;
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
                    obj.State(8:13) = 0;
                case robot.Direction.RESET
                    obj.reset();
                otherwise
                    return;
            end
        end

        function toggleGait(obj)
            %TOGGLEGAIT  Enable/disable trot gait (bound to 'g' key).
            %   When disabled, phases reset to home stance.
            obj.GaitEnabled = ~obj.GaitEnabled;
            if ~obj.GaitEnabled
                obj.GaitPhase = [0; 0.5; 0.5; 0];
                obj.GaitTimer = 0;
            end
        end

        function reset(obj)
            %RESET  Restore initial state and disable gait.
            reset@robot.Robot(obj);
            obj.GaitPhase = [0; 0.5; 0.5; 0];
            obj.GaitTimer = 0;
            obj.GaitEnabled = false;
        end

        function [verts, faces, edges] = buildGeometry(obj)
            %BUILDGEOMETRY  Wireframe for body box + shoulder/knee/foot vertices.
            %   Vertex layout:
            %     1-8:    body box
            %     9-12:   shoulder positions
            %     13-16:  knee positions (from solver)
            %     17-20:  foot positions (from solver)
            %   Edges: body wireframe edges + leg segments (shoulder→knee→foot).
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
            sv = [ bx,  sw, 0;  bx, -sw, 0; -bx,  sw, 0; -bx, -sw, 0];

            kv = obj.KneePositions;
            fv = obj.FootPositions;

            nBody = size(bv, 1);
            nShoulder = size(sv, 1);
            nKnee = size(kv, 1);

            verts = [bv; sv; kv; fv];
            faces = bf;

            le = zeros(8, 2);
            for i = 1:4
                le(2*i-1, :) = [nBody + i, nBody + nShoulder + i];
                le(2*i, :)   = [nBody + nShoulder + i, nBody + nShoulder + nKnee + i];
            end

            edges = [be; le];
        end

        function dstate = computeDynamics(obj, t, state, control)
            %COMPUTEDYNAMICS  Rigid-body dynamics with foot-ground contact.
            %   For each foot below z=0, applies a penalty-based spring-
            %   damper normal force and Coulomb friction.  Contact wrench
            %   is summed in the body frame.
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

            for i = 1:4
                foot_body = obj.FootPositions(i, :)';

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
            if any(~isfinite(dstate))
                dstate(:) = 0;
                warning('Non-finite dynamics detected — zeroing output.');
            end
        end

        function hg = plot(obj, ax)
            %PLOT  Build full quadruped visual: body, legs, shoulder/knee
            %       cylinders, red nose indicator.
            hg = plot@robot.Robot(obj, ax);
            [verts, faces, edges] = obj.buildGeometry();

            patch('Parent', hg, 'Vertices', verts, 'Faces', faces, ...
                  'FaceColor', [0.7 0.8 0.7], 'EdgeColor', 'none');

            obj.LegGraphics = cell(1, 4);
            nBody = 8;
            nShoulder = 4;
            sw = obj.shoulderWidth;
            bx = obj.bodyLength/2;
            shoulderPos = [bx, sw, 0; bx, -sw, 0; -bx, sw, 0; -bx, -sw, 0];
            for i = 1:4
                shoulderIdx = nBody + i;
                kneeIdx = nBody + nShoulder + i;
                footIdx = nBody + nShoulder + 4 + i;

                obj.LegGraphics{i}(1) = line('Parent', hg, ...
                    'XData', verts([shoulderIdx, kneeIdx], 1), ...
                    'YData', verts([shoulderIdx, kneeIdx], 2), ...
                    'ZData', verts([shoulderIdx, kneeIdx], 3), ...
                    'Color', 'k', 'LineWidth', 3);

                obj.LegGraphics{i}(2) = line('Parent', hg, ...
                    'XData', verts([kneeIdx, footIdx], 1), ...
                    'YData', verts([kneeIdx, footIdx], 2), ...
                    'ZData', verts([kneeIdx, footIdx], 3), ...
                    'Color', 'k', 'LineWidth', 3);

                obj.LegGraphics{i}(3) = line('Parent', hg, ...
                    'XData', verts(footIdx, 1), ...
                    'YData', verts(footIdx, 2), ...
                    'ZData', verts(footIdx, 3), ...
                    'Marker', '.', 'MarkerSize', 14, ...
                    'Color', [0.8 0.3 0.3]);

                [cx, cy, cz] = cylinder(0.018, 8);
                cz = cz * 0.04 - 0.02;
                sPos = shoulderPos(i, :);
                obj.LegGraphics{i}(4) = surf(cx + sPos(1), cy + sPos(2), cz + sPos(3), ...
                    'Parent', hg, ...
                    'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');

                kPos = obj.KneePositions(i, :);
                obj.LegGraphics{i}(5) = surf(cx + kPos(1), cy + kPos(2), cz + kPos(3), ...
                    'Parent', hg, ...
                    'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');
            end

            bz = obj.bodyHeight / 2;
            nose = [bx+0.05, 0, 0; bx+0.02, -bz, -bz; bx+0.02, bz, -bz; bx+0.02, 0, bz];
            obj.FrontIndicator = patch('Parent', hg, ...
                'Vertices', nose, 'Faces', [1,2,3; 1,3,4; 1,4,2; 2,4,3], ...
                'FaceColor', [0.9 0.1 0.1], 'EdgeColor', 'k');

            line('Parent', hg, ...
                'XData', [bx+0.05, bx+0.08], ...
                'YData', [0, 0], ...
                'ZData', [0, 0], ...
                'Color', [0.9 0.1 0.1], 'LineWidth', 2);
        end
    end

    methods (Access = protected)
        function n = getControlDim(obj)
            n = 6;
        end
    end

    methods (Access = private)
        function [theta1, theta2, theta3] = legIK(obj, footPos, shoulderPos)
            %LEGIK  Analytic inverse kinematics for 3-DOF leg.
            %   theta1 (hip yaw, about Z), theta2 (hip pitch, about Y),
            %   theta3 (knee pitch, about Y).  theta3=0 at full extension.
            r = footPos - shoulderPos;
            x = r(1); y = r(2); z = r(3);
            L1 = obj.legLength1;
            L2 = obj.legLength2;

            theta1 = atan2(y, x);

            d = sqrt(x^2 + y^2 + z^2);
            d = max(d, abs(L1 - L2) + 1e-12);
            d = min(d, L1 + L2 - 1e-12);

            cos_theta3 = (d^2 - L1^2 - L2^2) / (2 * L1 * L2);
            cos_theta3 = max(-1, min(1, cos_theta3));
            theta3 = acos(cos_theta3);

            theta2 = atan2(sqrt(x^2 + y^2), -z) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
        end

        function footPos = legFK(obj, theta1, theta2, theta3, shoulderPos)
            %LEGFK  Forward kinematics: joint angles → foot position.
            %   Returns footPos as 1×3 row vector in body frame.
            L1 = obj.legLength1;
            L2 = obj.legLength2;

            knee_local = [L1 * sin(theta2); 0; -L1 * cos(theta2)];
            foot_dir = [sin(theta2 + theta3); 0; -cos(theta2 + theta3)];

            Rz = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
            knee = Rz * knee_local;
            foot_local = Rz * (L2 * foot_dir);

            footPos = shoulderPos(:)' + knee(:)' + foot_local(:)';
        end

        function updateLegPositions(obj)
            %UPDATELEGPOSITIONS  Solve IK for all 4 legs, update joint
            %                    angles and knee/foot positions.
            sw = obj.shoulderWidth;
            bx = obj.bodyLength/2;
            L1 = obj.legLength1;
            L2 = obj.legLength2;
            shoulderPos = [bx, sw, 0; bx, -sw, 0; -bx, sw, 0; -bx, -sw, 0];

            for i = 1:4
                if obj.GaitEnabled
                    foot_target = obj.computeFootTarget(i);
                else
                    foot_target = shoulderPos(i, :) + [0, 0, -(L1 + L2)];
                end

                [t1, t2, t3] = obj.legIK(foot_target, shoulderPos(i, :));
                obj.JointAngles(i, :) = [t1, t2, t3];

                knee = [L1 * sin(t2), 0, -L1 * cos(t2)];
                Rz = [cos(t1), -sin(t1), 0; sin(t1), cos(t1), 0; 0, 0, 1];
                knee = (Rz * knee')';
                obj.KneePositions(i, :) = shoulderPos(i, :) + knee;
                obj.FootPositions(i, :) = obj.legFK(t1, t2, t3, shoulderPos(i, :));
            end
        end

        function footTarget = computeFootTarget(obj, legIdx)
            %COMPUTEFOOTTARGET  Gait trajectory for one foot over phase [0,1).
            %   Phase 0-0.5: swing (lift + forward).  Phase 0.5-1: stance
            %   (trailing back).  Sinusoidal vertical lift for smooth
            %   takeoff and landing.
            phase = obj.GaitPhase(legIdx);
            shoulder = obj.getShoulderPos(legIdx);
            defaultFoot = shoulder + [0, 0, -(obj.legLength1 + obj.legLength2)];

            if phase < 0.5
                t = phase / 0.5;
                dx = obj.StepLength * (t - 0.5);
                dz = obj.StepHeight * sin(pi * t);
            else
                t = (phase - 0.5) / 0.5;
                dx = obj.StepLength * (0.5 - t);
                dz = 0;
            end

            footTarget = defaultFoot + [dx, 0, dz];
        end

        function pos = getShoulderPos(obj, legIdx)
            %GETSHOULDERPOS  Shoulder position for leg index 1-4.
            bx = obj.bodyLength / 2;
            sw = obj.shoulderWidth;
            tbl = [bx,  sw, 0; bx, -sw, 0; -bx,  sw, 0; -bx, -sw, 0];
            pos = tbl(legIdx, :);
        end

        function updateWireframe(obj)
            %UPDATEWIREFRAME  Refresh 3D leg lines and knee cylinder
            %                 positions after IK solve.
            if isempty(obj.LegGraphics{1})
                return;
            end

            nBody = 8;
            nShoulder = 4;
            [verts, ~, ~] = obj.buildGeometry();

            for i = 1:4
                shoulderIdx = nBody + i;
                kneeIdx = nBody + nShoulder + i;
                footIdx = nBody + nShoulder + 4 + i;

                set(obj.LegGraphics{i}(1), ...
                    'XData', verts([shoulderIdx, kneeIdx], 1), ...
                    'YData', verts([shoulderIdx, kneeIdx], 2), ...
                    'ZData', verts([shoulderIdx, kneeIdx], 3));

                set(obj.LegGraphics{i}(2), ...
                    'XData', verts([kneeIdx, footIdx], 1), ...
                    'YData', verts([kneeIdx, footIdx], 2), ...
                    'ZData', verts([kneeIdx, footIdx], 3));

                set(obj.LegGraphics{i}(3), ...
                    'XData', verts(footIdx, 1), ...
                    'YData', verts(footIdx, 2), ...
                    'ZData', verts(footIdx, 3));

                [cx, cy, cz] = cylinder(0.018, 8);
                cz = cz * 0.04 - 0.02;
                kPos = obj.KneePositions(i, :);
                set(obj.LegGraphics{i}(5), ...
                    'XData', cx + kPos(1), ...
                    'YData', cy + kPos(2), ...
                    'ZData', cz + kPos(3));
            end
        end
    end
end
