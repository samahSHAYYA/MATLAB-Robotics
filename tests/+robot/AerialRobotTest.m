classdef AerialRobotTest < matlab.unittest.TestCase

    methods (Test)

        function constructorSetsControl(testCase)
            r = TestAerialRobot();
            testCase.verifyEqual(r.Control, zeros(4, 1));
        end

        function stepAdvancesState(testCase)
            r = TestAerialRobot();
            r.Control = [1; 1; 1; 1];
            s0 = r.State;
            r.step(0, 0.1);
            testCase.verifyNotEqual(r.State, s0);
        end

        function hoverZerosControl(testCase)
            r = TestAerialRobot();
            r.Control = [1; 2; 3; 4];
            r.hover();
            testCase.verifyEqual(r.Control, zeros(4, 1));
        end

    end

end

classdef TestAerialRobot < robot.AerialRobot
    methods
        function move(obj, direction, amount)
        end
        function [verts, faces, edges] = buildGeometry(obj)
            verts = [0 0 0];
            faces = [];
            edges = [];
        end
        function dstate = computeDynamics(obj, t, state, control)
            dstate = zeros(13, 1);
        end
    end
end
