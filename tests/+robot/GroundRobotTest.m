classdef GroundRobotTest < matlab.unittest.TestCase

    methods (Test)

        function constructorSetsControl(testCase)
            r = TestGroundRobot();
            testCase.verifyEqual(r.Control, zeros(2, 1));
        end

        function getControlDimReturns2(testCase)
            r = TestGroundRobot();
            testCase.verifyEqual(r.getControlDimPub(), 2);
        end

        function stepAdvancesState(testCase)
            r = TestGroundRobot();
            r.Control = [0.5; 0];
            s0 = r.State;
            r.step(0, 0.1);
            testCase.verifyNotEqual(r.State, s0);
        end

        function stepChangesPositionWithVelocity(testCase)
            r = TestGroundRobot();
            r.setState([0;0;0; 1;0;0;0; 1;0;0; 0;0;0]);
            r.Control = [0; 0];
            r.step(0, 0.1);
            testCase.verifyEqual(r.State(1), 0.1, 'AbsTol', 1e-4);
        end

    end

end

classdef TestGroundRobot < robot.GroundRobot
    methods
        function move(obj, direction, amount)
        end
        function [verts, faces, edges] = buildGeometry(obj)
            verts = [0 0 0];
            faces = [];
            edges = [];
        end
        function dstate = computeDynamics(obj, t, state, control)
            q = state(4:7);
            R = robot.Utils.quatToRotmx(q);
            dstate = [R * state(8:10); zeros(4,1); control(1); 0; 0; 0; 0; 0];
        end
        function n = getControlDimPub(obj)
            n = obj.getControlDim();
        end
    end
end
