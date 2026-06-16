classdef RobotFleetAppTest < matlab.unittest.TestCase
    properties
        App
    end

    methods (TestMethodTeardown)
        function cleanupApp(testCase)
            if ~isempty(testCase.App) && isvalid(testCase.App)
                testCase.App.stopSimulation();
                pause(0.1);
                if isvalid(testCase.App.Figure)
                    delete(testCase.App.Figure);
                end
                pause(0.1);
                delete(testCase.App);
            end
        end
    end

    methods
        function makeApp(testCase)
            app = RobotFleetApp();
            app.stopSimulation();
            pause(0.1);
            testCase.App = app;
        end
    end

    methods (Test)
        function constructAndDestroy(testCase)
            app = RobotFleetApp();
            testCase.App = app;
            testCase.assertTrue(isvalid(app));
            testCase.assertTrue(isvalid(app.Figure));
            testCase.assertEqual(numel(app.AxesHandle), 4);
            testCase.assertEqual(numel(app.AxesPanel), 4);
        end

        function spawnDifferentialDrive(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            testCase.assertFalse(isempty(app.Robots{1}));
            testCase.assertEqual(app.RobotVisible(1), true);
            testCase.assertEqual(app.RobotCounter, 1);
            testCase.assertEqual(app.Robots{1}.Id, "DifferentialDrive_1");
        end

        function spawnAllTypes(testCase)
            testCase.makeApp();
            app = testCase.App;
            types = {'DifferentialDrive', 'Quadcopter', 'Quadruped', 'Humanoid'};
            for i = 1:4
                app.spawnRobot(types{i});
                testCase.assertFalse(isempty(app.Robots{i}));
                testCase.assertEqual(app.Robots{i}.Id, string(sprintf('%s_%d', types{i}, i)));
            end
            testCase.assertEqual(app.RobotCounter, 4);
        end

        function spawnFailsWhenFull(testCase)
            testCase.makeApp();
            app = testCase.App;
            for i = 1:4
                app.spawnRobot('DifferentialDrive');
            end
            app.spawnRobot('DifferentialDrive');
            testCase.assertEqual(app.RobotCounter, 4);
        end

        function removeRobot(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            app.spawnRobot('Quadcopter');
            app.removeRobot(1);
            testCase.assertTrue(isempty(app.Robots{1}));
            testCase.assertEqual(app.RobotVisible(1), false);
            testCase.assertFalse(isempty(app.Robots{2}));
        end

        function selectRobot(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            app.selectRobot(1);
            testCase.assertEqual(app.SelectedIdx, 1);
        end

        function simStepRuns(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            t0 = app.SimTime;
            app.Running = true;
            for i = 1:20
                app.simStep();
            end
            app.Running = false;
            testCase.assertTrue(app.SimTime > t0);
        end

        function formationLine(testCase)
            testCase.makeApp();
            app = testCase.App;
            for i = 1:3
                app.spawnRobot('DifferentialDrive');
            end
            pos0 = cellfun(@(r) r.State(1), app.Robots(1:3));
            app.setFormation('line');
            pos1 = cellfun(@(r) r.State(1), app.Robots(1:3));
            testCase.assertNotEqual(pos0, pos1);
        end

        function formationGrid(testCase)
            testCase.makeApp();
            app = testCase.App;
            for i = 1:4
                app.spawnRobot('DifferentialDrive');
            end
            app.setFormation('grid');
        end

        function resetAll(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            app.Robots{1}.State(1) = 10;
            app.SimTime = 5;
            app.resetAll();
            testCase.assertEqual(app.SimTime, 0);
            testCase.assertEqual(app.Robots{1}.State(1), 0);
        end

        function sendCommand(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.spawnRobot('DifferentialDrive');
            app.selectRobot(1);
            app.sendCommand('FORWARD');
            app.simStep();
        end

        function toggleMode(testCase)
            testCase.makeApp();
            app = testCase.App;
            app.CtrlModeBtn.Value = 1;
            app.toggleMode();
            testCase.assertEqual(app.SyncMode, true);
            app.CtrlModeBtn.Value = 0;
            app.toggleMode();
            testCase.assertEqual(app.SyncMode, false);
        end

        function quatToRollPitch(testCase)
            testCase.makeApp();
            app = testCase.App;
            [r, p] = app.quatToRollPitch([1 0 0 0]);
            testCase.assertEqual(r, 0, 'AbsTol', 1e-10);
            testCase.assertEqual(p, 0, 'AbsTol', 1e-10);
            [r, p] = app.quatToRollPitch([cos(0.1) sin(0.1) 0 0]);
            testCase.assertEqual(r, 0.2, 'AbsTol', 1e-10);
        end

        function defaultParamsAllTypes(testCase)
            types = {'DifferentialDrive', 'Quadcopter', 'Quadruped', 'Humanoid'};
            for i = 1:4
                p = RobotFleetApp.defaultParams(types{i});
                testCase.assertFalse(isempty(p));
            end
        end
    end
end
