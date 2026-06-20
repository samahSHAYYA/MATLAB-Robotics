classdef (Sealed) Utils
    %UTILS  Rotation matrix, quaternion, and skew-symmetric helpers.
    %   All methods are Static — call as robot.Utils.* from outside
    %   the +robot package.
    methods (Static)
        function R = rotmx(axis, angle)
            %ROTMX  Elementary rotation matrix about x, y, or z.
            %   R = robot.Utils.rotmx(1, theta) returns 3x3 rotation for x-axis.
            %   Inputs:  axis  - 1 (x), 2 (y), or 3 (z)
            %            angle - rotation angle in radians
            %   Outputs: R     - 3x3 rotation matrix
            [~] = axis;
            c = cos(angle);
            s = sin(angle);
            switch axis
                case 1
                    R = [1, 0, 0; 0, c, -s; 0, s, c];
                case 2
                    R = [c, 0, s; 0, 1, 0; -s, 0, c];
                case 3
                    R = [c, -s, 0; s, c, 0; 0, 0, 1];
                otherwise
                    error('axis must be 1 (x), 2 (y), or 3 (z)');
            end
        end

        function qOut = quatMultiply(q1, q2)
            %QUATMULTIPLY  Hamilton product of two quaternions.
            %   qOut = robot.Utils.quatMultiply(q1, q2)
            %   Inputs:  q1, q2 - [w,x,y,z]' unit quaternions
            %   Outputs: qOut   - [w,x,y,z]' = q1 * q2
            w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
            w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
            qOut = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
                    w1*x2 + x1*w2 + y1*z2 - z1*y2;
                    w1*y2 - x1*z2 + y1*w2 + z1*x2;
                    w1*z2 + x1*y2 - y1*x2 + z1*w2];
        end

        function R = quatToRotmx(q)
            %QUATTOROTMX  Quaternion [w,x,y,z]' to 3x3 rotation matrix.
            %   R = robot.Utils.quatToRotmx(q)
            %   Uses point convention (rotates points, not frames).
            %   Inputs:  q - [w,x,y,z]' unit quaternion
            %   Outputs: R - 3x3 rotation matrix
            n = norm(q);
            if n < 1e-15
                R = eye(3);
                return;
            end
            w = q(1)/n; x = q(2)/n; y = q(3)/n; z = q(4)/n;
            xx = x*x; yy = y*y; zz = z*z;
            xy = x*y; xz = x*z; yz = y*z;
            wx = w*x; wy = w*y; wz = w*z;
            R = [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy);
                 2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx);
                 2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)];
        end

        function [roll, pitch, yaw] = rotmxToRPY(R)
            %ROTMXTORPY  Extract roll-pitch-yaw (rad) from rotation matrix.
            %   [roll,pitch,yaw] = robot.Utils.rotmxToRPY(R)
            %   Uses ZYX convention: yaw (z) → pitch (y) → roll (x).
            %   Inputs:  R - 3x3 rotation matrix
            %   Outputs: roll, pitch, yaw - angles in radians
            roll = atan2(R(3,2), R(3,3));
            pitch = -asin(R(3,1));
            yaw = atan2(R(2,1), R(1,1));
        end

        function q = rpyToQuat(roll, pitch, yaw)
            %RPYTOQUAT  Roll-pitch-yaw (rad) to quaternion [w,x,y,z]'.
            %   q = robot.Utils.rpyToQuat(roll, pitch, yaw)
            %   Uses ZYX convention: yaw (z) → pitch (y) → roll (x).
            %   Inputs:  roll, pitch, yaw - angles in radians
            %   Outputs: q - [w,x,y,z]' unit quaternion
            cr = cos(roll/2); sr = sin(roll/2);
            cp = cos(pitch/2); sp = sin(pitch/2);
            cy = cos(yaw/2); sy = sin(yaw/2);
            q = [cr*cp*cy + sr*sp*sy;
                 sr*cp*cy - cr*sp*sy;
                 cr*sp*cy + sr*cp*sy;
                 cr*cp*sy - sr*sp*cy];
        end

        function S = skew(v)
            %SKEW  Skew-symmetric cross-product matrix of 3-vector.
            %   S = robot.Utils.skew(v)  satisfies  S * a = cross(v, a).
            %   Inputs:  v - 3-element vector
            %   Outputs: S - 3x3 skew-symmetric matrix
            S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        end

        function S = crossEquivalent(v)
            %CROSSEQUIVALENT  Alias for skew(v).
            S = robot.Utils.skew(v);
        end
    end
end
