classdef RobotTest < matlab.unittest.TestCase

    methods (Test)

        function constructorSetsStateAndControl(testCase)
            r = robot.TestRobot();
            testCase.verifyEqual(r.State, [0;0;0;1;0;0;0;0;0;0;0;0;0]);
            testCase.verifyEmpty(r.Control);
        end

        function getSetStateRoundTrip(testCase)
            r = robot.TestRobot();
            newState = [1;2;3; 0;1;0;0; 4;5;6; 0.1;0.2;0.3];
            r.setState(newState);
            testCase.verifyEqual(r.getState(), newState);
        end

        function resetRestoresInitialState(testCase)
            r = robot.TestRobot();
            newState = [1;2;3; 0;1;0;0; 4;5;6; 0.1;0.2;0.3];
            r.setState(newState);
            r.reset();
            testCase.verifyEqual(r.State, r.InitialState);
        end

        function plotReturnsHandle(testCase)
            fig = figure('Visible', 'off');
            ax = axes('Parent', fig);
            r = robot.TestRobot();
            hg = r.plot(ax);
            testCase.verifyTrue(ishandle(hg));
            testCase.verifyEqual(hg.Type, 'hgtransform');
            close(fig);
        end

        function abstractMethodsDoNotThrow(testCase)
            r = robot.TestRobot();
            r.move(robot.Direction.FORWARD, 1);
            [v, f, e] = r.buildGeometry();
            d = r.computeDynamics(0, r.State, []);
            testCase.verifyEqual(size(v), [1, 3]);
            testCase.verifyEmpty(f);
            testCase.verifyEmpty(e);
            testCase.verifyEqual(d, zeros(13, 1));
        end

        function stepNoOpOnBaseRobot(testCase)
            r = robot.TestRobot();
            s0 = r.State;
            r.step(0, 0.01);
            testCase.verifyEqual(r.State, s0);
        end

    end

end
