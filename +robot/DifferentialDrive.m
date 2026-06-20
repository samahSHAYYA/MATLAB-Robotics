classdef DifferentialDrive < robot.GroundRobot
    %DIFFERENTIALDRIVE  Planar 2-DOF wheeled robot (x-translation + yaw).
    %   Control: [torque_left; torque_right] applied at wheels.
    %   Dynamics models net forward force from wheel torques and yaw
    %   torque from differential wheel speeds.

    properties
        wheelRadius (1,1) double
        trackWidth  (1,1) double
        mass        (1,1) double
        inertia     (1,1) double
        maxTorque   (1,1) double
        BodyGraphics (1,:) cell
    end

    methods
        function obj = DifferentialDrive(params)
            %DIFFERENTIALDRIVE  Construct from parameter struct.
            %   Fields: params.geometric.{wheelRadius, trackWidth}
            %           params.dynamic.{mass, inertia, maxTorque}
            arguments
                params (1,1) struct
            end
            obj@robot.GroundRobot();
            obj.wheelRadius = params.geometric.wheelRadius;
            obj.trackWidth = params.geometric.trackWidth;
            obj.mass = params.dynamic.mass;
            obj.inertia = params.dynamic.inertia;
            obj.maxTorque = params.dynamic.maxTorque;
        end

        function move(obj, direction, amount)
            %MOVE  Set left/right wheel torques from direction command.
            %   FORWARD: both wheels positive.  BACKWARD: both negative.
            %   LEFT/YAW_LEFT: wheels counter-rotate (spin left).
            %   RIGHT/YAW_RIGHT: wheels counter-rotate (spin right).
            %   STOP: zero torque.
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
                    obj.State(8:13) = 0;
                case robot.Direction.RESET
                    obj.reset();
                otherwise
            end
        end

        function [verts, faces, edges] = buildGeometry(obj, hg)
            r = obj.wheelRadius;
            tw = obj.trackWidth;

            bx = 0.15; by = 0.10; bz = 0.04;
            cz = 0.04;
            bv = [-bx, -by, -bz+cz;  bx, -by, -bz+cz;  bx,  by, -bz+cz; -bx,  by, -bz+cz;
                  -bx, -by,  bz+cz;  bx, -by,  bz+cz;  bx,  by,  bz+cz; -bx,  by,  bz+cz];
            bf = [1, 2, 3, 4; 5, 8, 7, 6; 1, 5, 6, 2; 3, 7, 8, 4; 1, 4, 8, 5; 2, 6, 7, 3];
            be = [1,2; 2,3; 3,4; 4,1; 5,6; 6,7; 7,8; 8,5; 1,5; 2,6; 3,7; 4,8];
            verts = bv; faces = bf; edges = be;

            if nargin >= 2
                bodyColor = [0.91 0.30 0.24];
                wheelColor = [0.80 0.25 0.20];
                handles = cell(1, 8);
                nxt = 1;

                handles{nxt} = patch('Parent', hg, 'Vertices', bv, 'Faces', bf, ...
                    'FaceColor', bodyColor, 'EdgeColor', 'k', 'LineWidth', 1.5);
                nxt = nxt + 1;

                nPts = 24;
                th = (0:nPts-1) * 2*pi / nPts;
                for side = [-1, 1]
                    wy = side * tw/2;
                    wz = r;
                    handles{nxt} = patch('Parent', hg, ...
                        'XData', r*cos(th), 'YData', wy + zeros(1,nPts), ...
                        'ZData', r*sin(th) + wz, ...
                        'FaceColor', 'none', 'EdgeColor', wheelColor, 'LineWidth', 2);
                    nxt = nxt + 1;
                    handles{nxt} = line('Parent', hg, ...
                        'XData', [-r, r], 'YData', [wy, wy], 'ZData', [wz, wz], ...
                        'Color', wheelColor, 'LineWidth', 1);
                    nxt = nxt + 1;
                    handles{nxt} = line('Parent', hg, ...
                        'XData', [0, 0], 'YData', [wy, wy], 'ZData', [wz-r, wz+r], ...
                        'Color', wheelColor, 'LineWidth', 1);
                    nxt = nxt + 1;
                end

                handles{nxt} = line('Parent', hg, ...
                    'XData', [0, 0], 'YData', [-tw/2, tw/2], 'ZData', [r, r], ...
                    'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);

                obj.BodyGraphics = handles;
            end
        end

        function dstate = computeDynamics(obj, ~, state, control)
            %COMPUTEDYNAMICS  Planar drive: forward force + yaw torque.
            %   F_drive = (tau_L + tau_R) / r_wheel
            %   T_yaw   = (tau_R - tau_L) * trackWidth / (2 * r_wheel)
            %   State: only vx (body x) and wz (z-axis) are non-zero.
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

        function hg = plot(obj, ax)
            hg = plot@robot.Robot(obj, ax);
            obj.buildGeometry(hg);

            axH = ancestor(ax, 'axes');
            nPts = 24;
            th = (0:nPts-1) * 2*pi / nPts;
            rx = 0.10; ry = 0.08;
            cx = rx * cos(th); cy = ry * sin(th);
            obj.ShadowHandle = patch(axH, cx, cy, zeros(1,nPts), ...
                [0.3 0.3 0.3], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
            set(obj.ShadowHandle, 'UserData', struct('baseX', cx, 'baseY', cy));

            obj.TrailHandle = line(axH, NaN, NaN, NaN, ...
                'Color', [0.91 0.30 0.24], 'LineWidth', 1.5);

            bx = 0.15; by = 0.10;
            lr = 0.015; ln = 8; lt = (0:ln-1)*2*pi/ln;
            obj.RunningLightHandles = cell(1, 4);
            for si = [-1, 1]
                idx = (si+3)/2*2;
                obj.RunningLightHandles{idx-1} = patch(axH, ...
                    bx*0.5 + lr*cos(lt), si*by*0.5 + lr*sin(lt), zeros(1,ln)+0.04, ...
                    [1 0.7 0.1], 'EdgeColor', 'none', 'Visible', 'off');
                obj.RunningLightHandles{idx} = patch(axH, ...
                    -bx*0.5 + lr*cos(lt), si*by*0.5 + lr*sin(lt), zeros(1,ln)+0.04, ...
                    [0.9 0.1 0.1], 'EdgeColor', 'none', 'Visible', 'off');
            end
        end
    end
end
