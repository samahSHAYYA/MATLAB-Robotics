classdef HumanoidTest < matlab.unittest.TestCase

    properties
        params
    end

    methods (TestClassSetup)
        function setupParams(testCase)
            p = struct();
            p.geometric.bodyHeight = 0.8;
            p.geometric.bodyWidth = 0.4;
            p.geometric.hipWidth = 0.2;
            p.kinematic.thighLength = 0.3;
            p.kinematic.shinLength = 0.3;
            p.kinematic.footLength = 0.22;
            p.dynamic.mass = 30;
            p.dynamic.inertia = diag([0.5, 0.8, 0.4]);
            p.elastic.k_contact = 8000;
            p.elastic.b_contact = 80;
            p.elastic.mu = 0.9;
            p.balance.gainP = 1000;
            p.balance.gainD = 120;
            testCase.params = p;
        end
    end

    methods (Test)

        function constructorAcceptsValidParams(testCase)
            r = robot.Humanoid(testCase.params);
            testCase.verifyClass(r, 'robot.Humanoid');
            testCase.verifyEqual(r.Control, zeros(6, 1));
        end

        function constructorValidatesParams(testCase)
            bad = testCase.params;
            bad.kinematic.thighLength = -1;
            testCase.verifyError(@() robot.Humanoid(bad), 'MATLAB:expectedPositive');
        end

        function moveForwardSetsControlYPositive(testCase)
            r = robot.Humanoid(testCase.params);
            r.move(robot.Direction.FORWARD, 1);
            testCase.verifyGreaterThan(r.Control(2), 0);
        end

        function moveBackwardSetsControlYNegative(testCase)
            r = robot.Humanoid(testCase.params);
            r.move(robot.Direction.BACKWARD, 1);
            testCase.verifyLessThan(r.Control(2), 0);
        end

        function moveYawLeftSetsControl6Positive(testCase)
            r = robot.Humanoid(testCase.params);
            r.move(robot.Direction.YAW_LEFT, 1);
            testCase.verifyGreaterThan(r.Control(6), 0);
        end

        function getControlDimReturns6(testCase)
            r = robot.Humanoid(testCase.params);
            testCase.verifyEqual(r.Control, zeros(6, 1));
        end

        function toggleGaitFlipsEnabled(testCase)
            r = robot.Humanoid(testCase.params);
            testCase.verifyFalse(r.GaitEnabled);
            r.toggleGait();
            testCase.verifyTrue(r.GaitEnabled);
            r.toggleGait();
            testCase.verifyFalse(r.GaitEnabled);
        end

        function stepAdvancesState(testCase)
            r = robot.Humanoid(testCase.params);
            s0 = r.State;
            r.step(0, 0.01);
            testCase.verifyNotEqual(r.State, s0);
        end

        function buildGeometryReturnsValid(testCase)
            r = robot.Humanoid(testCase.params);
            [v, f, e] = r.buildGeometry();
            testCase.verifyEqual(size(v, 2), 3);
            testCase.verifyEqual(size(v, 1), 8);
            testCase.verifyEqual(size(f, 1), 6);
            testCase.verifyEqual(size(e, 1), 12);
        end

        function resetRestoresStateAndDisablesGait(testCase)
            r = robot.Humanoid(testCase.params);
            r.GaitEnabled = true;
            r.Control = rand(6, 1);
            r.reset();
            testCase.verifyEqual(r.State, r.InitialState);
            testCase.verifyFalse(r.GaitEnabled);
            testCase.verifyEqual(r.GaitPhase, [0; 0.5]);
        end

        function legIKStraightDownReturnsZeroAngles(testCase)
            r = robot.Humanoid(testCase.params);
            r.step(0, 0.01);
            theta = r.JointAngles;
            testCase.verifyEqual(theta, zeros(2, 3), 'AbsTol', 1e-4);
        end

        function computeDynamicsReturns13Element(testCase)
            r = robot.Humanoid(testCase.params);
            dstate = r.computeDynamics(0, r.State, r.Control);
            testCase.verifyEqual(size(dstate), [13, 1]);
        end

        function balanceControllerZeroNetTorqueUpright(testCase)
            r = robot.Humanoid(testCase.params);
            state = [0; 0; 5; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            control = zeros(6, 1);
            dstate = r.computeDynamics(0, state, control);
            testCase.verifyEqual(dstate(11:13), zeros(3, 1), 'AbsTol', 1e-6);
        end

    end

end
