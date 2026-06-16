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
            q1_obj = quaternion(q1(1), q1(2), q1(3), q1(4));
            q2_obj = quaternion(q2(1), q2(2), q2(3), q2(4));
            qOut_obj = q1_obj * q2_obj;
            qOut = compact(qOut_obj)';
        end

        function R = quatToRotmx(q)
            %QUATTOROTMX  Quaternion [w,x,y,z]' to 3x3 rotation matrix.
            %   R = robot.Utils.quatToRotmx(q)
            %   Uses point convention (rotates points, not frames).
            %   Inputs:  q - [w,x,y,z]' unit quaternion
            %   Outputs: R - 3x3 rotation matrix
            q_obj = quaternion(q(1), q(2), q(3), q(4));
            R = rotmat(q_obj, 'point');
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
