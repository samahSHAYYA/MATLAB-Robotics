classdef QuadrupedTest < matlab.unittest.TestCase

    properties
        params
    end

    methods (TestClassSetup)
        function setupParams(testCase)
            p = struct();
            p.geometric.bodyLength = 0.4;
            p.geometric.bodyWidth = 0.2;
            p.geometric.bodyHeight = 0.1;
            p.geometric.shoulderWidth = 0.12;
            p.kinematic.legLength1 = 0.15;
            p.kinematic.legLength2 = 0.15;
            p.dynamic.mass = 3;
            p.dynamic.inertia = diag([0.015, 0.04, 0.05]);
            p.elastic.k_contact = 5000;
            p.elastic.b_contact = 50;
            p.elastic.mu = 0.9;
            testCase.params = p;
        end
    end

    methods (Test)

        function constructorAcceptsValidParams(testCase)
            r = robot.Quadruped(testCase.params);
            testCase.verifyClass(r, 'robot.Quadruped');
            testCase.verifyEqual(r.Control, zeros(6, 1));
        end

        function constructorValidatesParams(testCase)
            bad = testCase.params;
            bad.dynamic.mass = -1;
            testCase.verifyError(@() robot.Quadruped(bad), 'MATLAB:expectedPositive');
        end

        function moveForwardSetsControlXPositive(testCase)
            r = robot.Quadruped(testCase.params);
            r.move(robot.Direction.FORWARD, 1);
            testCase.verifyGreaterThan(r.Control(1), 0);
        end

        function moveBackwardSetsControlXNegative(testCase)
            r = robot.Quadruped(testCase.params);
            r.move(robot.Direction.BACKWARD, 1);
            testCase.verifyLessThan(r.Control(1), 0);
        end

        function moveYawLeftSetsControl6Positive(testCase)
            r = robot.Quadruped(testCase.params);
            r.move(robot.Direction.YAW_LEFT, 1);
            testCase.verifyGreaterThan(r.Control(6), 0);
        end

        function moveYawRightSetsControl6Negative(testCase)
            r = robot.Quadruped(testCase.params);
            r.move(robot.Direction.YAW_RIGHT, 1);
            testCase.verifyLessThan(r.Control(6), 0);
        end

        function toggleGaitFlipsEnabled(testCase)
            r = robot.Quadruped(testCase.params);
            testCase.verifyFalse(r.GaitEnabled);
            r.toggleGait();
            testCase.verifyTrue(r.GaitEnabled);
            r.toggleGait();
            testCase.verifyFalse(r.GaitEnabled);
        end

        function getControlDimReturns6(testCase)
            r = robot.Quadruped(testCase.params);
            testCase.verifyEqual(r.Control, zeros(6, 1));
        end

        function stepAdvancesState(testCase)
            r = robot.Quadruped(testCase.params);
            s0 = r.State;
            r.step(0, 0.01);
            testCase.verifyNotEqual(r.State, s0);
        end

        function buildGeometryReturnsCorrectSizes(testCase)
            r = robot.Quadruped(testCase.params);
            [v, f, e] = r.buildGeometry();
            testCase.verifyEqual(size(v), [20, 3]);
            testCase.verifyEqual(size(f), [6, 4]);
            testCase.verifyEqual(size(e), [20, 2]);
        end

        function resetRestoresStateAndDisablesGait(testCase)
            r = robot.Quadruped(testCase.params);
            r.Control = rand(6, 1);
            r.GaitEnabled = true;
            r.reset();
            testCase.verifyEqual(r.State, r.InitialState);
            testCase.verifyFalse(r.GaitEnabled);
            testCase.verifyEqual(r.GaitPhase, [0; 0.5; 0.5; 0]);
        end

        function legIKStraightDownReturnsZeroAngles(testCase)
            r = robot.Quadruped(testCase.params);
            r.step(0, 0.01);
            theta = r.JointAngles;
            testCase.verifyEqual(theta, zeros(4, 3), 'AbsTol', 1e-10);
        end

        function legFKIKRoundTrip(testCase)
            r = robot.Quadruped(testCase.params);
            r.step(0, 0.01);
            bx = r.bodyLength / 2;
            sw = r.shoulderWidth;
            L1 = r.legLength1;
            L2 = r.legLength2;
            expectedFoot = [bx, sw, -(L1+L2); bx, -sw, -(L1+L2); -bx, sw, -(L1+L2); -bx, -sw, -(L1+L2)];
            testCase.verifyEqual(r.FootPositions, expectedFoot, 'AbsTol', 1e-10);
        end

        function computeDynamicsReturns13Element(testCase)
            r = robot.Quadruped(testCase.params);
            dstate = r.computeDynamics(0, r.State, r.Control);
            testCase.verifyEqual(size(dstate), [13, 1]);
        end

    end

end
