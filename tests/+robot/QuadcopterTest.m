classdef QuadcopterTest < matlab.unittest.TestCase

    properties
        params
    end

    methods (TestClassSetup)
        function setupParams(testCase)
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            p.dynamic.kTorque = 0.1;
            testCase.params = p;
        end
    end

    methods (Test)

        function constructorAcceptsValidParams(testCase)
            r = robot.Quadcopter(testCase.params);
            testCase.verifyClass(r, 'robot.Quadcopter');
            testCase.verifyEqual(r.Control, ones(4,1) * (0.5 * 9.81 / 4), 'AbsTol', 1e-10);
        end

        function moveForwardIncreasesPitchTarget(testCase)
            r = robot.Quadcopter(testCase.params);
            r.move(robot.Direction.FORWARD, 1);
            r.step(0, 0.01);
            testCase.verifyNotEqual(r.Control, ones(4,1) * (0.5 * 9.81 / 4));
        end

        function moveUpIncreasesThrottle(testCase)
            r = robot.Quadcopter(testCase.params);
            r.move(robot.Direction.UP, 1);
            r.step(0, 0.01);
            testCase.verifyGreaterThan(sum(r.Control), 0.5 * 9.81);
        end

        function moveYawLeftChangesControl(testCase)
            r = robot.Quadcopter(testCase.params);
            r.move(robot.Direction.YAW_LEFT, 1);
            r.step(0, 0.01);
            testCase.verifyNotEqual(r.Control, ones(4,1) * (0.5 * 9.81 / 4));
        end

        function buildGeometryReturnsValid(testCase)
            r = robot.Quadcopter(testCase.params);
            [v, f, e] = r.buildGeometry();
            testCase.verifyEqual(size(v, 2), 3);
            testCase.verifyTrue(size(v, 1) > 8);
            testCase.verifyTrue(size(f, 1) >= 6);
            testCase.verifyTrue(size(e, 1) >= 12);
        end

        function computeDynamicsReturns13Element(testCase)
            r = robot.Quadcopter(testCase.params);
            dstate = r.computeDynamics(0, r.State, r.Control);
            testCase.verifyEqual(size(dstate), [13, 1]);
        end

        function stepAdvancesState(testCase)
            r = robot.Quadcopter(testCase.params);
            r.State(8:10) = [1; 0; 0];
            s0 = r.State;
            r.step(0, 0.01);
            testCase.verifyNotEqual(r.State, s0);
        end

        function resetRestoresInitialState(testCase)
            r = robot.Quadcopter(testCase.params);
            r.move(robot.Direction.FORWARD, 1);
            r.step(0, 0.01);
            r.reset();
            testCase.verifyEqual(r.State, r.InitialState);
            testCase.verifyEqual(r.Control, ones(4,1) * (0.5 * 9.81 / 4), 'AbsTol', 1e-10);
        end

        function computeAttitudeThrustsReturns4Element(testCase)
            r = robot.Quadcopter(testCase.params);
            T = r.computeAttitudeThrusts();
            testCase.verifyEqual(size(T), [4, 1]);
            testCase.verifyTrue(all(T >= 0));
            testCase.verifyTrue(all(T <= r.maxThrust));
        end

        function hoverMaintainsAltitude(testCase)
            r = robot.Quadcopter(testCase.params);
            r.move(robot.Direction.UP, 0);
            r.step(0, 0.01);
            r.step(0, 0.01);
            testCase.verifyEqual(r.State(3), 0.5, 'AbsTol', 1e-6);
        end

    end

end
