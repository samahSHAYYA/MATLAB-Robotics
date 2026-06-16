classdef DynamicsEngineTest < matlab.unittest.TestCase

    methods (Test)

        function rk4StepZeroDynamics(testCase)
            dynFun = @(t, s, u) zeros(13, 1);
            s0 = zeros(13, 1);
            s0(4) = 1;
            s1 = robot.DynamicsEngine.rk4Step(dynFun, 0, s0, [], 0.01);
            testCase.verifyEqual(s1, s0, 'AbsTol', 1e-12);
        end

        function rk4StepConstantVelocity(testCase)
            dynFun = @(t, s, u) [s(8); zeros(12, 1)];
            s0 = [0; 0; 0; 1; 0; 0; 0; 1; 0; 0; 0; 0; 0];
            dt = 0.1;
            s1 = robot.DynamicsEngine.rk4Step(dynFun, 0, s0, [], dt);
            testCase.verifyEqual(s1(1), dt, 'AbsTol', 1e-6);
            testCase.verifyEqual(s1(8), 1.0, 'AbsTol', 1e-12);
        end

        function rk4StepHarmonicOscillator(testCase)
            dynFun = @(t, s, u) [s(8); 0; 0; 0; 0; 0; 0; -s(1); 0; 0; 0; 0; 0];
            s0 = [1; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            dt = 0.01;
            s = s0; t = 0;
            for i = 1:1000
                s = robot.DynamicsEngine.rk4Step(dynFun, t, s, [], dt);
                t = t + dt;
            end
            E0 = 0.5 * s0(1)^2 + 0.5 * s0(8)^2;
            E = 0.5 * s(1)^2 + 0.5 * s(8)^2;
            testCase.verifyEqual(E, E0, 'RelTol', 1e-3);
        end

        function integrateMatchesRk4Step(testCase)
            dynFun = @(t, s, u) [s(8); zeros(12, 1)];
            s0 = [0; 0; 0; 1; 0; 0; 0; 0.5; 0; 0; 0; 0; 0];
            dt = 0.05;
            states = robot.DynamicsEngine.integrate(dynFun, [0, 1], s0, [], dt);
            sManual = s0; t = 0;
            for k = 1:size(states, 1)-1
                sManual = robot.DynamicsEngine.rk4Step(dynFun, t, sManual, [], dt);
                t = t + dt;
            end
            testCase.verifyEqual(states(end, :)', sManual, 'AbsTol', 1e-12);
            testCase.verifyEqual(states(1, :)', s0);
            testCase.verifyTrue(size(states, 1) > 1);
        end

    end

end
