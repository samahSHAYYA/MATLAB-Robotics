classdef UtilsTest < matlab.unittest.TestCase

    methods (Test)

        function rotmxReturns3x3(testCase)
            R = robot.Utils.rotmx(1, 0.5);
            testCase.verifyEqual(size(R), [3, 3]);
        end

        function rotmxXrotatesYtoZ(testCase)
            R = robot.Utils.rotmx(1, pi/2);
            y = [0; 1; 0];
            yRot = R * y;
            testCase.verifyEqual(yRot, [0; 0; 1], 'AbsTol', 1e-12);
        end

        function rotmxYrotatesZtoX(testCase)
            R = robot.Utils.rotmx(2, pi/2);
            z = [0; 0; 1];
            zRot = R * z;
            testCase.verifyEqual(zRot, [1; 0; 0], 'AbsTol', 1e-12);
        end

        function rotmxZrotatesXtoY(testCase)
            R = robot.Utils.rotmx(3, pi/2);
            x = [1; 0; 0];
            xRot = R * x;
            testCase.verifyEqual(xRot, [0; 1; 0], 'AbsTol', 1e-12);
        end

        function quatMultiplyIdentity(testCase)
            id = [1; 0; 0; 0];
            q = [0.7071; 0.7071; 0; 0];
            result = robot.Utils.quatMultiply(q, id);
            testCase.verifyEqual(result, q, 'AbsTol', 1e-10);
        end

        function quatMultiplyTwoKnown(testCase)
            q180x = [0; 1; 0; 0];
            result = robot.Utils.quatMultiply(q180x, q180x);
            testCase.verifyEqual(abs(result), [1; 0; 0; 0], 'AbsTol', 1e-10);
        end

        function quatToRotmxOrthonormal(testCase)
            q = [0.7071; 0; 0.7071; 0];
            R = robot.Utils.quatToRotmx(q);
            testCase.verifyEqual(det(R), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(R' * R, eye(3), 'AbsTol', 1e-10);
        end

        function rotmxToRPYRoundTrip(testCase)
            angles = [0.3; -0.2; 0.5];
            Rx = robot.Utils.rotmx(1, angles(1));
            Ry = robot.Utils.rotmx(2, angles(2));
            Rz = robot.Utils.rotmx(3, angles(3));
            R = Rz * Ry * Rx;
            [r, p, y] = robot.Utils.rotmxToRPY(R);
            testCase.verifyEqual([r; p; y], angles, 'AbsTol', 1e-12);
        end

        function skewIsSkewSymmetric(testCase)
            v = [1; 2; 3];
            S = robot.Utils.skew(v);
            testCase.verifyEqual(S, -S');
            testCase.verifyEqual(diag(S), zeros(3, 1));
        end

        function crossEquivalentEqualsSkew(testCase)
            v = [4; -1; 7];
            S1 = robot.Utils.skew(v);
            S2 = robot.Utils.crossEquivalent(v);
            testCase.verifyEqual(S1, S2);
        end

    end

end
