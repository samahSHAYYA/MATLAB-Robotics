classdef (Sealed) Utils
    methods (Static)
        function R = rotmx(axis, angle)
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
            q1_obj = quaternion(q1(1), q1(2), q1(3), q1(4));
            q2_obj = quaternion(q2(1), q2(2), q2(3), q2(4));
            qOut_obj = q1_obj * q2_obj;
            qOut = compact(qOut_obj)';
        end

        function R = quatToRotmx(q)
            q_obj = quaternion(q(1), q(2), q(3), q(4));
            R = rotmat(q_obj, 'point');
        end

        function [roll, pitch, yaw] = rotmxToRPY(R)
            roll = atan2(R(3,2), R(3,3));
            pitch = -asin(R(3,1));
            yaw = atan2(R(2,1), R(1,1));
        end

        function S = skew(v)
            S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        end

        function S = crossEquivalent(v)
            S = robot.Utils.skew(v);
        end
    end
end
