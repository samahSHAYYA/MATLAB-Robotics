classdef RobotFleetApp < handle
    properties
        Figure              matlab.ui.Figure
        MainGrid            matlab.ui.container.GridLayout
        ViewportGrid        matlab.ui.container.GridLayout
        AxesPanel           (:,1)
        AxesHandle          (:,1) matlab.graphics.axis.Axes
        Visualizer          (:,1) cell
        Robots              cell
        BBoxHandles         cell
        BBoxVisible         (:,1) logical
        RobotVisible        (:,1) logical
        SelectedIdx         (1,1) double = 0
        LegendCheckboxes    struct
        BBoxCheckboxes      struct
        CtrlModeBtn         matlab.ui.control.ToggleButton
        TelemetryGrid       matlab.ui.container.GridLayout
        TelemetryLabels     struct
        StatusLabel         matlab.ui.control.Label
        FPSLabel            matlab.ui.control.Label
        SimTimer            timer
        PhysicsDt           (1,1) double = 0.005
        RenderDt            (1,1) double = 0.033
        Running             (1,1) logical = false
        SimTime             (1,1) double = 0
        DesiredDirection    robot.Direction = robot.Direction.STOP
        DesiredAmount       (1,1) double = 0
        SyncMode            (1,1) logical = false
        Pool                parallel.Pool
        PoolAvailable       (1,1) logical = false
        ScriptSchedule      struct
        ScriptIdx           (1,1) double = 0
        ScriptMode          (1,1) logical = false
        ScriptLabel         matlab.ui.control.Label
        ColorPalette        (1,4) cell = {[0.91 0.30 0.24], [0.20 0.60 0.86], [0.18 0.80 0.44], [0.61 0.35 0.71]}
        RobotCounter        (1,1) double = 0
    end

    methods
        function app = RobotFleetApp()
            app.buildUI();
            app.tryStartPool();
            app.startSimulation();
        end

        function buildUI(app)
            app.Figure = uifigure('Name', 'Robot Fleet Command Center', ...
                'NumberTitle', 'off', 'Color', [0.94 0.94 0.94], ...
                'Position', [50 50 1400 850], ...
                'WindowKeyPressFcn', @app.onKeyPress, ...
                'CloseRequestFcn', @app.onClose);
            app.MainGrid = uigridlayout(app.Figure, [7 3], ...
                'RowHeight', {40, '1x', '1x', '1x', 120, 80, 25}, ...
                'ColumnWidth', {200, '1x', 220}, ...
                'Padding', [8 8 8 8], 'RowSpacing', 6, 'ColumnSpacing', 8);

            titleLabel = uilabel(app.MainGrid, 'Text', 'Robot Fleet Command Center', ...
                'FontSize', 18, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center');
            titleLabel.Layout.Row = 1; titleLabel.Layout.Column = [1 3];

            app.buildSpawnPanel();
            app.buildViewportGrid();
            app.buildControlPanel();
            app.buildLegendPanel();
            app.buildTelemetryPanel();
            app.buildStatusBar();
        end

        function buildSpawnPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Robot Fleet', 'FontWeight', 'bold');
            p.Layout.Row = [2 3]; p.Layout.Column = 1;
            gl = uigridlayout(p, [6 1], 'RowHeight', {25, 25, 25, 25, 25, '1x'}, ...
                'Padding', [5 5 5 5], 'RowSpacing', 4);

            dd = uidropdown(gl, 'Items', {'DifferentialDrive', 'Quadcopter', 'Quadruped', 'Humanoid'});
            btnAdd = uibutton(gl, 'push', 'Text', '+ Spawn', ...
                'ButtonPushedFcn', @(~,~) app.spawnRobot(dd.Value));
            btnRemove = uibutton(gl, 'push', 'Text', '- Remove Selected', ...
                'ButtonPushedFcn', @(~,~) app.removeRobot(app.SelectedIdx));
            btnLoad = uibutton(gl, 'push', 'Text', 'Load Script CSV...', ...
                'ButtonPushedFcn', @(~,~) app.loadScript());
            app.ScriptLabel = uilabel(gl, 'Text', '', 'FontSize', 11, ...
                'FontColor', [0.3 0.3 0.3]);
            app.ScriptLabel.Visible = 'off';
            uilabel(gl, 'Text', 'Click a viewport to select robot');
        end

        function buildViewportGrid(app)
            vp = uipanel(app.MainGrid, 'Title', 'Viewports', 'FontWeight', 'bold');
            vp.Layout.Row = [2 5]; vp.Layout.Column = 2;
            app.ViewportGrid = uigridlayout(vp, [2 2], ...
                'RowHeight', {'1x', '1x'}, 'ColumnWidth', {'1x', '1x'}, ...
                'Padding', [4 4 4 4], 'RowSpacing', 6, 'ColumnSpacing', 6);

            app.AxesPanel = gobjects(4,1);
            app.AxesHandle = gobjects(4,1);
            app.Visualizer = cell(4,1);
            app.Robots = cell(4,1);
            app.BBoxHandles = cell(4,1);
            app.BBoxVisible = false(4,1);
            app.RobotVisible = false(4,1);

            for i = 1:4
                [r, c] = ind2sub([2 2], i);
                ax = uiaxes(app.ViewportGrid);
                ax.Layout.Row = r; ax.Layout.Column = c;
                ax.Visible = 'off';
                ax.ButtonDownFcn = @(~,~) app.selectRobot(i);
                app.AxesHandle(i) = ax;
                app.AxesPanel(i) = uipanel(app.ViewportGrid, 'Title', '', ...
                    'BorderType', 'line', 'BackgroundColor', [1 1 1]);
                app.AxesPanel(i).Layout.Row = r;
                app.AxesPanel(i).Layout.Column = c;
                app.AxesHandle(i).Parent = app.AxesPanel(i);
                app.AxesHandle(i).Position = [10 10 1 1];
            end
        end

        function buildLegendPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Legend', 'FontWeight', 'bold');
            p.Layout.Row = 4; p.Layout.Column = 1;
            app.LegendGrid = uigridlayout(p, [6 3], ...
                'RowHeight', {22, 22, 22, 22, 22, '1x'}, ...
                'ColumnWidth', {22, 22, '1x'}, ...
                'Padding', [4 4 4 4], 'RowSpacing', 2);
            uilabel(app.LegendGrid, 'Text', '', 'FontSize', 10);
            uilabel(app.LegendGrid, 'Text', '', 'FontSize', 10);
            uilabel(app.LegendGrid, 'Text', 'Robots', 'FontSize', 10, 'FontWeight', 'bold');
            for i = 1:4
                visCb = uicheckbox(app.LegendGrid, 'Value', 0, ...
                    'ValueChangedFcn', @(~,~) app.toggleVisibility(i));
                visCb.Layout.Row = 1+i; visCb.Layout.Column = 1;
                bboxCb = uicheckbox(app.LegendGrid, 'Value', 0, ...
                    'ValueChangedFcn', @(~,~) app.toggleBoundingBox(i));
                bboxCb.Layout.Row = 1+i; bboxCb.Layout.Column = 2;
                lbl = uilabel(app.LegendGrid, 'Text', '─', ...
                    'FontSize', 10, 'FontColor', [0.5 0.5 0.5]);
                lbl.Layout.Row = 1+i; lbl.Layout.Column = 3;
                app.LegendCheckboxes(i).vis = visCb;
                app.LegendCheckboxes(i).bbox = bboxCb;
                app.LegendCheckboxes(i).label = lbl;
            end
        end

        function buildControlPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Control', 'FontWeight', 'bold');
            p.Layout.Row = [2 3]; p.Layout.Column = 3;
            gl = uigridlayout(p, [7 3], ...
                'RowHeight', {25, 35, 35, 35, 30, 30, '1x'}, ...
                'ColumnWidth', {'1x', '1x', '1x'}, ...
                'Padding', [6 6 6 6], 'RowSpacing', 4);

            app.CtrlModeBtn = uibutton(gl, 'state', 'Text', 'Mode: Individual', ...
                'Value', 0, 'ValueChangedFcn', @app.toggleMode);
            app.CtrlModeBtn.Layout.Row = 1; app.CtrlModeBtn.Layout.Column = [1 3];

            labels = {'↑', '↺', '↻', '←', 'STOP', '→', '↓', '⎋', '⏻'};
            cmds = {'FORWARD', 'YAW_LEFT', 'YAW_RIGHT', 'LEFT', 'STOP', ...
                'RIGHT', 'BACKWARD', 'UP', 'DOWN'};
            positions = {[2 2], [2 1], [2 3], [3 1], [3 2], [3 3], [4 2], [4 1], [4 3]};
            for j = 1:9
                btn = uibutton(gl, 'push', 'Text', labels{j}, ...
                    'FontSize', 11, 'FontWeight', 'bold', ...
                    'ButtonPushedFcn', @(~,~) app.sendCommand(cmd{j}));
                btn.Layout.Row = positions{j}{1}; btn.Layout.Column = positions{j}{2};
            end

            uibutton(gl, 'push', 'Text', 'Formation: Line', ...
                'ButtonPushedFcn', @(~,~) app.setFormation('line'));
            uibutton(gl, 'push', 'Text', 'Formation: Grid', ...
                'ButtonPushedFcn', @(~,~) app.setFormation('grid'));
            uibutton(gl, 'push', 'Text', 'Reset All', ...
                'ButtonPushedFcn', @(~,~) app.resetAll());
            [~,idx] = sort([2 3 1]); % layout row 5-6
            btns = gl.Children(end-2:end);
            for j = 1:3
                btns(j).Layout.Row = 5+idx(j); btns(j).Layout.Column = [1 3];
            end
        end

        function buildTelemetryPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Telemetry', 'FontWeight', 'bold');
            p.Layout.Row = 5; p.Layout.Column = 3;
            app.TelemetryGrid = uigridlayout(p, [7 2], ...
                'RowHeight', repmat({18}, 1, 7), ...
                'ColumnWidth', {60, '1x'}, ...
                'Padding', [6 4 6 4], 'RowSpacing', 1);
            rows = {'Pos X', 'Pos Y', 'Pos Z', 'Vel', 'Omega', 'Roll', 'Pitch'};
            flds = {'posX', 'posY', 'posZ', 'vel', 'omega', 'roll', 'pitch'};
            for j = 1:7
                uilabel(app.TelemetryGrid, 'Text', rows{j}+':', ...
                    'FontSize', 10, 'FontWeight', 'bold');
                app.TelemetryLabels.(flds{j}) = uilabel(app.TelemetryGrid, ...
                    'Text', '─', 'FontSize', 10);
                [app.TelemetryLabels.(flds{j}).Layout.Row, ~] = deal(j, 2);
            end
        end

        function buildStatusBar(app)
            app.StatusLabel = uilabel(app.MainGrid, 'Text', 'Ready', ...
                'FontSize', 11, 'FontColor', [0.4 0.4 0.4]);
            app.StatusLabel.Layout.Row = 7; app.StatusLabel.Layout.Column = 1;
            app.FPSLabel = uilabel(app.MainGrid, 'Text', '', ...
                'FontSize', 11, 'FontColor', [0.4 0.4 0.4]);
            app.FPSLabel.Layout.Row = 7; app.FPSLabel.Layout.Column = 3;
            poolLbl = uilabel(app.MainGrid, 'Text', '', ...
                'FontSize', 11, 'FontColor', [0.4 0.4 0.4]);
            poolLbl.Layout.Row = 7; poolLbl.Layout.Column = 2;
            app.StatusLabel.Parent.Children(end).Text = 'Pool: checking...';
            app.StatusLabel.Parent.Children(end).HorizontalAlignment = 'center';
        end

        function tryStartPool(app)
            try
                if ~exist('gcp', 'file')
                    app.PoolAvailable = false;
                    app.updateStatus('Parallel pool: not available');
                    return;
                end
                pool = gcp('nocreate');
                if isempty(pool)
                    evalc('parpool(''Threads'', 4);');
                    pool = gcp('nocreate');
                end
                app.Pool = pool;
                app.PoolAvailable = ~isempty(pool);
                if app.PoolAvailable
                    app.updateStatus(sprintf('Parallel pool: %d workers', pool.NumWorkers));
                else
                    app.updateStatus('Parallel pool: failed');
                end
            catch ME
                app.PoolAvailable = false;
                app.updateStatus('Parallel pool: ' + ME.message);
            end
        end

        function spawnRobot(app, type)
            n = find(cellfun(@isempty, app.Robots), 1);
            if isempty(n)
                app.updateStatus('All viewports occupied');
                return;
            end
            app.RobotCounter = app.RobotCounter + 1;
            params = app.defaultParams(type);
            switch type
                case 'DifferentialDrive'; r = robot.DifferentialDrive(params);
                case 'Quadcopter';         r = robot.Quadcopter(params);
                case 'Quadruped';          r = robot.Quadruped(params);
                case 'Humanoid';           r = robot.Humanoid(params);
            end
            r.Id = sprintf('%s_%d', type, app.RobotCounter);
            app.Robots{n} = r;
            app.RobotVisible(n) = true;
            ax = app.AxesHandle(n);
            ax.Visible = 'on';
            ax.Position = [25 25 1 1];
            ax.ButtonDownFcn = @(~,~) app.selectRobot(n);
            cla(ax); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); view(ax, 3);
            ax.Projection = 'perspective';
            xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
            xlim(ax, [-1.5 1.5]); ylim(ax, [-1.5 1.5]); zlim(ax, [-0.5 2.0]);

            light(ax, 'Position', [1 1 3], 'Style', 'infinite');
            light(ax, 'Position', [-1 -1 1], 'Style', 'infinite');

            gx = [-5 5 5 -5]; gy = [-5 -5 5 5]; gz = [0 0 0 0];
            patch(ax, gx, gy, gz, [0.85 0.85 0.85]);

            vg = robot.Visualizer(ax);
            vg.addRobot(r);
            app.Visualizer{n} = vg;
            app.BBoxHandles{n} = [];
            app.BBoxVisible(n) = false;
            app.LegendCheckboxes(n).vis.Enable = 'on';
            app.LegendCheckboxes(n).bbox.Enable = 'on';
            app.LegendCheckboxes(n).label.Text = r.Id;
            c = app.ColorPalette{mod(n-1,4)+1};
            app.LegendCheckboxes(n).label.FontColor = c;
            app.AxesPanel(n).Title = r.Id;
            app.selectRobot(n);
            app.updateStatus(sprintf('Spawned %s at viewport %d', r.Id, n));
            drawnow;
        end

        function removeRobot(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            r = app.Robots{idx};
            delete(r.GraphicsTransform.Children);
            delete(r.GraphicsTransform);
            app.Robots{idx} = [];
            app.Visualizer{idx} = [];
            app.BBoxHandles{idx} = [];
            app.RobotVisible(idx) = false;
            app.BBoxVisible(idx) = false;
            app.AxesHandle(idx).Visible = 'off';
            app.AxesPanel(idx).Title = '';
            app.LegendCheckboxes(idx).vis.Enable = 'off';
            app.LegendCheckboxes(idx).vis.Value = 0;
            app.LegendCheckboxes(idx).bbox.Enable = 'off';
            app.LegendCheckboxes(idx).bbox.Value = 0;
            app.LegendCheckboxes(idx).label.Text = '─';
            app.LegendCheckboxes(idx).label.FontColor = [0.5 0.5 0.5];
            if app.SelectedIdx == idx
                app.SelectedIdx = 0;
            end
            app.updateStatus(sprintf('Removed %s', r.Id));
        end

        function selectRobot(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            if app.SelectedIdx > 0 && app.SelectedIdx <= numel(app.AxesPanel)
                app.AxesPanel(app.SelectedIdx).BorderType = 'line';
                app.AxesPanel(app.SelectedIdx).HighlightColor = [0.5 0.5 0.5];
            end
            app.SelectedIdx = idx;
            app.AxesPanel(idx).BorderType = 'line';
            app.AxesPanel(idx).HighlightColor = [0 0.45 0.74];
            app.updateTelemetry();
        end

        function startSimulation(app)
            app.Running = true;
            app.SimTimer = timer('ExecutionMode', 'fixedRate', ...
                'Period', app.RenderDt, ...
                'TimerFcn', @(~,~) app.simStep());
            start(app.SimTimer);
        end

        function stopSimulation(app)
            app.Running = false;
            if isvalid(app.SimTimer)
                stop(app.SimTimer);
                delete(app.SimTimer);
            end
        end

        function simStep(app)
            if ~app.Running; return; end
            tStart = tic;
            nSteps = ceil(app.RenderDt / app.PhysicsDt);
            active = find(~cellfun(@isempty, app.Robots));
            if app.ScriptMode && ~isempty(active)
                app.playbackStep(nSteps);
            end
            for k = 1:nSteps
                for i = active(:)'
                    r = app.Robots{i};
                    if ~app.ScriptMode
                        dir = app.DesiredDirection;
                        amt = app.DesiredAmount;
                        if app.SyncMode || i == app.SelectedIdx
                            r.move(dir, amt);
                        end
                    end
                    r.step(app.SimTime, app.PhysicsDt);
                end
                app.SimTime = app.SimTime + app.PhysicsDt;
            end
            for i = active(:)'
                if app.RobotVisible(i)
                    app.Visualizer{i}.update(app.Robots{i});
                    app.updateRobotGraphics(i);
                end
                if app.BBoxVisible(i) && app.RobotVisible(i)
                    app.drawBoundingBox(i);
                elseif app.BBoxVisible(i)
                    app.hideBoundingBox(i);
                end
            end
            app.updateTelemetry();
            elapsed = toc(tStart);
            fps = 1 / max(elapsed, 0.001);
            app.FPSLabel.Text = sprintf('FPS: %.0f | Sim: %.1fs', fps, app.SimTime);
        end

        function updateRobotGraphics(~, idx)
            if nargin < 2; return; end
        end

        function drawBoundingBox(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            r = app.Robots{idx};
            [obbVerts, obbEdges] = robot.Collision.buildOBB(r);
            ax = app.AxesHandle(idx);
            if ~isempty(app.BBoxHandles{idx}) && isvalid(app.BBoxHandles{idx})
                delete(app.BBoxHandles{idx});
            end
            hold(ax, 'on');
            c = app.ColorPalette{mod(idx-1,4)+1} * 1.5;
            c = min(c, 1);
            h = gobjects(size(obbEdges,1), 1);
            for e = 1:size(obbEdges,1)
                h(e) = plot3(ax, obbVerts(obbEdges(e,:),1), ...
                    obbVerts(obbEdges(e,:),2), ...
                    obbVerts(obbEdges(e,:),3), ...
                    'Color', c, 'LineWidth', 1.5, 'LineStyle', '--');
            end
            app.BBoxHandles{idx} = h;
        end

        function hideBoundingBox(app, idx)
            if ~isempty(app.BBoxHandles{idx})
                if isvalid(app.BBoxHandles{idx})
                    delete(app.BBoxHandles{idx});
                end
                app.BBoxHandles{idx} = [];
            end
        end

        function toggleVisibility(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            app.RobotVisible(idx) = app.LegendCheckboxes(idx).vis.Value;
            r = app.Robots{idx};
            if app.RobotVisible(idx)
                set(r.GraphicsTransform, 'Visible', 'on');
            else
                set(r.GraphicsTransform, 'Visible', 'off');
                app.hideBoundingBox(idx);
                app.LegendCheckboxes(idx).bbox.Value = 0;
                app.BBoxVisible(idx) = false;
            end
        end

        function toggleBoundingBox(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            app.BBoxVisible(idx) = app.LegendCheckboxes(idx).bbox.Value;
            if app.BBoxVisible(idx)
                if ~app.RobotVisible(idx)
                    app.LegendCheckboxes(idx).vis.Value = 1;
                    app.toggleVisibility(idx);
                end
                app.drawBoundingBox(idx);
            else
                app.hideBoundingBox(idx);
            end
        end

        function updateTelemetry(app)
            if app.SelectedIdx < 1 || isempty(app.Robots{app.SelectedIdx})
                flds = fieldnames(app.TelemetryLabels);
                for j = 1:numel(flds)
                    app.TelemetryLabels.(flds{j}).Text = '─';
                end
                return;
            end
            r = app.Robots{app.SelectedIdx};
            s = r.State;
            app.TelemetryLabels.posX.Text = sprintf('%.2f', s(1));
            app.TelemetryLabels.posY.Text = sprintf('%.2f', s(2));
            app.TelemetryLabels.posZ.Text = sprintf('%.2f', s(3));
            app.TelemetryLabels.vel.Text = sprintf('%.2f', norm(s(8:10)));
            app.TelemetryLabels.omega.Text = sprintf('%.2f', norm(s(11:13)));
            [roll, pitch] = app.quatToRollPitch(s(4:7));
            app.TelemetryLabels.roll.Text = sprintf('%.1f°', rad2deg(roll));
            app.TelemetryLabels.pitch.Text = sprintf('%.1f°', rad2deg(pitch));
        end

        function [roll, pitch] = quatToRollPitch(~, q)
            w = q(1); x = q(2); y = q(3); z = q(4);
            roll = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
            pitch = asin(2*(w*y - z*x));
        end

        function toggleMode(app, ~, ~)
            app.SyncMode = app.CtrlModeBtn.Value;
            if app.SyncMode
                app.CtrlModeBtn.Text = 'Mode: Sync';
                app.updateStatus('Synchronize mode: all robots move together');
            else
                app.CtrlModeBtn.Text = 'Mode: Individual';
                app.updateStatus('Individual mode: selected robot only');
            end
        end

        function sendCommand(app, cmdStr)
            dir = robot.Direction.(cmdStr);
            amt = 1.0;
            if any(strcmp(cmdStr, {'STOP', 'YAW_LEFT', 'YAW_RIGHT'}))
                amt = 0.5;
            end
            if app.SyncMode
                app.DesiredDirection = dir;
                app.DesiredAmount = amt;
                for i = find(~cellfun(@isempty, app.Robots))'
                    app.Robots{i}.move(dir, amt);
                end
            elseif app.SelectedIdx > 0
                app.DesiredDirection = dir;
                app.DesiredAmount = amt;
                app.Robots{app.SelectedIdx}.move(dir, amt);
            end
        end

        function setFormation(app, type)
            active = find(~cellfun(@isempty, app.Robots));
            n = numel(active);
            if n < 2; app.updateStatus('Need at least 2 robots for formation'); return; end
            for k = 1:n
                r = app.Robots{active(k)};
                switch type
                    case 'line'
                        pos = [(k-1)*0.8 - (n-1)*0.4, 0, r.State(3)];
                    case 'grid'
                        [r2, c2] = ind2sub([ceil(sqrt(n)), ceil(sqrt(n))], k);
                        pos = [(c2-1)*0.8 - 0.4, (r2-1)*0.8 - 0.4, r.State(3)];
                end
                r.State(1:3) = pos(:);
            end
            app.updateStatus(sprintf('Formation: %s (%d robots)', type, n));
        end

        function resetAll(app)
            for i = find(~cellfun(@isempty, app.Robots))'
                app.Robots{i}.reset();
            end
            app.SimTime = 0;
            app.updateStatus('All robots reset');
        end

        function loadScript(app)
            [file, path] = uigetfile({'*.csv', 'CSV files'}, 'Select script file');
            if file == 0; return; end
            try
                data = readtable(fullfile(path, file));
                required = {'time', 'robot_id', 'command', 'amount'};
                if ~all(ismember(required, lower(data.Properties.VariableNames)))
                    app.updateStatus('CSV must have columns: time, robot_id, command, amount');
                    return;
                end
                data.Properties.VariableNames = lower(data.Properties.VariableNames);
                app.ScriptSchedule = data;
                app.ScriptIdx = 1;
                app.ScriptMode = true;
                app.ScriptLabel.Text = sprintf('Script: %s', file);
                app.ScriptLabel.Visible = 'on';
                app.updateStatus(sprintf('Loaded script: %s (%d commands)', file, height(data)));
            catch ME
                app.updateStatus('Script load failed: ' + ME.message);
            end
        end

        function playbackStep(app, nSteps)
            if ~app.ScriptMode || isempty(app.ScriptSchedule)
                return;
            end
            data = app.ScriptSchedule;
            tEnd = app.SimTime + nSteps * app.PhysicsDt;
            while app.ScriptIdx <= height(data) && data.time(app.ScriptIdx) <= tEnd
                row = data(app.ScriptIdx, :);
                for i = find(~cellfun(@isempty, app.Robots))'
                    if strcmp(char(app.Robots{i}.Id), char(row.robot_id))
                        try
                            dir = robot.Direction.(upper(char(row.command)));
                            amt = row.amount;
                            app.Robots{i}.move(dir, amt);
                        catch
                        end
                    end
                end
                app.ScriptIdx = app.ScriptIdx + 1;
            end
            if app.ScriptIdx > height(data)
                app.ScriptMode = false;
                app.ScriptLabel.Visible = 'off';
                app.updateStatus('Script playback complete');
            end
        end

        function onKeyPress(app, ~, evt)
            if app.ScriptMode; return; end
            map = containers.Map();
            map('uparrow')    = {'FORWARD', 1.0};
            map('downarrow')  = {'BACKWARD', 1.0};
            map('leftarrow')  = {'LEFT', 0.5};
            map('rightarrow') = {'RIGHT', 0.5};
            map('space')      = {'STOP', 0};
            map('r')          = {'RESET', 0};
            map('g')          = {'TOGGLE_GAIT', 0};
            map('w')          = {'UP', 1.0};
            map('s')          = {'DOWN', 1.0};
            map('a')          = {'ROLL_LEFT', 0.5};
            map('d')          = {'ROLL_RIGHT', 0.5};
            map('q')          = {'PITCH_UP', 0.5};
            map('e')          = {'PITCH_DOWN', 0.5};
            if isKey(map, evt.Key)
                cmd = map(evt.Key);
                if strcmp(cmd{1}, 'RESET')
                    for i = find(~cellfun(@isempty, app.Robots))'
                        app.Robots{i}.reset();
                    end
                    return;
                end
                if strcmp(cmd{1}, 'TOGGLE_GAIT')
                    for i = find(~cellfun(@isempty, app.Robots))'
                        if isprop(app.Robots{i}, 'GaitEnabled')
                            app.Robots{i}.GaitEnabled = ~app.Robots{i}.GaitEnabled;
                        end
                    end
                    return;
                end
                app.sendCommand(cmd{1});
            end
        end

        function onClose(app, ~, ~)
            app.stopSimulation();
            delete(app.Figure);
        end

        function updateStatus(app, msg)
            if isvalid(app.StatusLabel)
                app.StatusLabel.Text = char(msg);
            end
        end
    end

    methods (Static)
        function params = defaultParams(type)
            switch type
                case 'DifferentialDrive'
                    params.geometric.wheelRadius = 0.05;
                    params.geometric.trackWidth = 0.2;
                    params.dynamic.mass = 2.0;
                    params.dynamic.inertia = 0.05;
                    params.dynamic.maxTorque = 5.0;
                case 'Quadcopter'
                    params.geometric.armLength = 0.2;
                    params.geometric.bodySize = [0.1 0.1 0.05];
                    params.dynamic.mass = 0.5;
                    params.dynamic.inertia = diag([0.002 0.002 0.004]);
                    params.dynamic.maxThrust = 2.0;
                    params.dynamic.kTorque = 0.05;
                case 'Quadruped'
                    params.geometric.bodyLength = 0.4;
                    params.geometric.bodyWidth = 0.2;
                    params.geometric.bodyHeight = 0.1;
                    params.kinematic.legLength1 = 0.15;
                    params.kinematic.legLength2 = 0.15;
                    params.geometric.shoulderWidth = 0.12;
                    params.dynamic.mass = 3.0;
                    params.dynamic.inertia = diag([0.015 0.04 0.05]);
                    params.elastic.k_contact = 5000;
                    params.elastic.b_contact = 50;
                    params.elastic.mu = 0.8;
                case 'Humanoid'
                    params.geometric.bodyHeight = 0.8;
                    params.geometric.bodyWidth = 0.4;
                    params.geometric.hipWidth = 0.2;
                    params.kinematic.thighLength = 0.35;
                    params.kinematic.shinLength = 0.35;
                    params.kinematic.footLength = 0.22;
                    params.dynamic.mass = 30.0;
                    params.dynamic.inertia = diag([0.5 1.2 1.0]);
                    params.elastic.k_contact = 8000;
                    params.elastic.b_contact = 80;
                    params.elastic.mu = 0.9;
                    params.balance.gainP = 1000;
                    params.balance.gainD = 120;
            end
        end
    end
end
