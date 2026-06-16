classdef AerialRobotTest < matlab.unittest.TestCase

    methods (Test)

        function constructorSetsControl(testCase)
            r = robot.TestAerialRobot();
            testCase.verifyEqual(r.Control, zeros(4, 1));
        end

        function stepWithZeroDynamicsPreservesState(testCase)
            r = robot.TestAerialRobot();
            r.Control = [1; 1; 1; 1];
            s0 = r.State;
            r.step(0, 0.1);
            testCase.verifyEqual(r.State, s0, 'AbsTol', 1e-15);
        end

        function hoverZerosControl(testCase)
            r = robot.TestAerialRobot();
            r.Control = [1; 2; 3; 4];
            r.hover();
            testCase.verifyEqual(r.Control, zeros(4, 1));
        end

    end

end
