classdef DifferentialDriveTest < matlab.unittest.TestCase

    properties
        params
    end

    methods (TestClassSetup)
        function setupParams(testCase)
            p = struct();
            p.geometric.wheelRadius = 0.05;
            p.geometric.trackWidth = 0.2;
            p.dynamic.mass = 1.0;
            p.dynamic.inertia = 0.01;
            p.dynamic.maxTorque = 0.5;
            testCase.params = p;
        end
    end

    methods (Test)

        function constructorAcceptsValidParams(testCase)
            r = robot.DifferentialDrive(testCase.params);
            testCase.verifyClass(r, 'robot.DifferentialDrive');
            testCase.verifyEqual(r.Control, zeros(2, 1));
        end

        function moveForwardSetsEqualPositiveTorques(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.move(robot.Direction.FORWARD, 1);
            testCase.verifyEqual(r.Control(1), r.Control(2));
            testCase.verifyGreaterThan(r.Control(1), 0);
        end

        function moveBackwardSetsEqualNegativeTorques(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.move(robot.Direction.BACKWARD, 1);
            testCase.verifyEqual(r.Control(1), r.Control(2));
            testCase.verifyLessThan(r.Control(1), 0);
        end

        function moveLeftSetsLeftLessThanRight(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.move(robot.Direction.LEFT, 1);
            testCase.verifyLessThan(r.Control(1), r.Control(2));
        end

        function moveRightSetsLeftGreaterThanRight(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.move(robot.Direction.RIGHT, 1);
            testCase.verifyGreaterThan(r.Control(1), r.Control(2));
        end

        function buildGeometryReturnsValid(testCase)
            r = robot.DifferentialDrive(testCase.params);
            [v, f, e] = r.buildGeometry();
            testCase.verifyEqual(size(v, 2), 3);
            testCase.verifyTrue(size(v, 1) >= 8);
            testCase.verifyTrue(size(f, 1) >= 6);
            testCase.verifyTrue(size(e, 1) >= 12);
        end

        function computeDynamicsReturns13Element(testCase)
            r = robot.DifferentialDrive(testCase.params);
            dstate = r.computeDynamics(0, r.State, r.Control);
            testCase.verifyEqual(size(dstate), [13, 1]);
        end

        function computeDynamicsWithDriveChangesState(testCase)
            r = robot.DifferentialDrive(testCase.params);
            dstate = r.computeDynamics(0, r.State, [0.5; 0.5]);
            testCase.verifyGreaterThan(abs(dstate(8)), 0);
        end

        function stepAdvancesState(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.Control = [0.1; 0.1];
            s0 = r.State;
            r.step(0, 0.05);
            testCase.verifyNotEqual(r.State, s0);
        end

        function resetRestoresInitialState(testCase)
            r = robot.DifferentialDrive(testCase.params);
            r.Control = [0.5; 0.5];
            r.setState([1;0;0; 0;1;0;0; 0.5;0;0; 0;0;0]);
            r.reset();
            testCase.verifyEqual(r.State, r.InitialState);
            testCase.verifyEqual(r.Control, zeros(2, 1));
        end

    end

end
