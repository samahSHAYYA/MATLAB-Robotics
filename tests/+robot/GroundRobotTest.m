classdef GroundRobotTest < matlab.unittest.TestCase

    methods (Test)

        function constructorSetsControl(testCase)
            r = robot.TestGroundRobot();
            testCase.verifyEqual(r.Control, zeros(2, 1));
        end

        function getControlDimReturns2(testCase)
            r = robot.TestGroundRobot();
            testCase.verifyEqual(r.getControlDimPub(), 2);
        end

        function stepAdvancesState(testCase)
            r = robot.TestGroundRobot();
            r.Control = [0.5; 0];
            s0 = r.State;
            r.step(0, 0.1);
            testCase.verifyNotEqual(r.State, s0);
        end

        function stepChangesPositionWithVelocity(testCase)
            r = robot.TestGroundRobot();
            r.setState([0;0;0; 1;0;0;0; 1;0;0; 0;0;0]);
            r.Control = [0; 0];
            r.step(0, 0.1);
            testCase.verifyEqual(r.State(1), 0.1, 'AbsTol', 1e-4);
        end

    end

end
