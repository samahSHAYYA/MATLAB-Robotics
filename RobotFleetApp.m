classdef RobotFleetApp < handle
    properties
        Figure              matlab.ui.Figure
        MainGrid            matlab.ui.container.GridLayout
        ScenePanel
        SceneAxes
        SceneVisualizer     = []
        Robots              cell
        BBoxHandles         cell
        BBoxVisible         (:,1) logical
        RobotVisible        (:,1) logical
        SelectedIdx         (1,1) double = 0
        LegendGrid
        LegendCheckboxes    struct
        TargetDropdown
        TelemetryGrid       matlab.ui.container.GridLayout
        TelemetryLabels     struct = struct()
        StatusLabel
        FPSLabel
        PoolLabel
        SimTimer            timer
        PhysicsDt           (1,1) double = 0.005
        RenderDt            (1,1) double = 0.033
        Running             (1,1) logical = false
        SimTime             (1,1) double = 0
        DesiredDirection    robot.Direction = robot.Direction.STOP
        DesiredAmount       (1,1) double = 0
        Pool
        PoolTimer
        PoolAvailable       (1,1) logical = false
        ScriptSchedule
        ScriptIdx           (1,1) double = 0
        ScriptMode          (1,1) logical = false
        ScriptLabel
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
                'RowHeight', {40, '1x', '1x', '1x', 120, 80, 40}, ...
                'ColumnWidth', {200, '1x', 220}, ...
                'Padding', [8 8 8 8], 'RowSpacing', 6, 'ColumnSpacing', 8);

            titleLabel = uilabel(app.MainGrid, 'Text', 'Robot Fleet Command Center', ...
                'FontSize', 18, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center');
            titleLabel.Layout.Row = 1; titleLabel.Layout.Column = [1 3];

            app.buildSpawnPanel();
            app.buildScene();
            app.buildControlPanel();
            app.buildLegendPanel();
            app.buildTelemetryPanel();
            app.buildStatusBar();
        end

        function buildSpawnPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Robot Fleet', 'FontWeight', 'bold');
            p.Layout.Row = [2 3]; p.Layout.Column = 1;
            gl = uigridlayout(p, [7 1], 'RowHeight', {25, 25, 25, 25, 25, 25, '1x'}, ...
                'Padding', [5 5 5 5], 'RowSpacing', 4);

            dd = uidropdown(gl, 'Items', {'DifferentialDrive', 'Quadcopter', 'Quadruped', 'Humanoid'});
            uibutton(gl, 'push', 'Text', '+ Spawn', ...
                'ButtonPushedFcn', @(~,~) app.spawnRobot(dd.Value));
            uibutton(gl, 'push', 'Text', '+ Spawn (Custom...)', ...
                'ButtonPushedFcn', @(~,~) app.spawnRobotCustom(dd.Value));
            uibutton(gl, 'push', 'Text', '- Remove Selected', ...
                'ButtonPushedFcn', @(~,~) app.removeRobot(app.SelectedIdx));
            uibutton(gl, 'push', 'Text', 'Load Script CSV...', ...
                'ButtonPushedFcn', @(~,~) app.loadScript());
            app.ScriptLabel = uilabel(gl, 'Text', '', 'FontSize', 11, ...
                'FontColor', [0.3 0.3 0.3]);
            app.ScriptLabel.Visible = 'off';
            uilabel(gl, 'Text', 'Click legend to select robot');
        end

        function buildScene(app)
            app.ScenePanel = uipanel(app.MainGrid, 'Title', 'Scene', ...
                'FontWeight', 'bold');
            app.ScenePanel.Layout.Row = [2 5]; app.ScenePanel.Layout.Column = 2;
            ig = uigridlayout(app.ScenePanel, [1 1], ...
                'Padding', [0 0 0 0], 'RowSpacing', 0, 'ColumnSpacing', 0);
            app.SceneAxes = uiaxes(ig);
            app.SceneAxes.Layout.Row = 1; app.SceneAxes.Layout.Column = 1;
            app.SceneAxes.Visible = 'on';
            hold(app.SceneAxes, 'on');
            axis(app.SceneAxes, 'equal');
            grid(app.SceneAxes, 'on');
            view(app.SceneAxes, 3);
            app.SceneAxes.Projection = 'perspective';
            xlabel(app.SceneAxes, 'X'); ylabel(app.SceneAxes, 'Y'); zlabel(app.SceneAxes, 'Z');
            xlim(app.SceneAxes, [-2.5 2.5]);
            ylim(app.SceneAxes, [-2.5 2.5]);
            zlim(app.SceneAxes, [-0.5 2.5]);
            light(app.SceneAxes, 'Position', [1 1 3], 'Style', 'infinite');
            light(app.SceneAxes, 'Position', [-1 -1 1], 'Style', 'infinite');

            app.Robots = cell(4,1);
            app.BBoxHandles = cell(4,1);
            app.BBoxVisible = false(4,1);
            app.RobotVisible = false(4,1);
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
                app.addLegendCheckbox(i);
            end
        end

        function buildControlPanel(app)
            p = uipanel(app.MainGrid, 'Title', 'Control', 'FontWeight', 'bold');
            p.Layout.Row = [2 3]; p.Layout.Column = 3;
            gl = uigridlayout(p, [7 3], ...
                'RowHeight', {25, 35, 35, 35, 30, 30, '1x'}, ...
                'ColumnWidth', {'1x', '1x', '1x'}, ...
                'Padding', [6 6 6 6], 'RowSpacing', 4);

            app.TargetDropdown = uidropdown(gl, ...
                'Items', {'ALL', 'R1', 'R2', 'R3', 'R4'}, ...
                'Value', 'ALL', ...
                'Tooltip', 'Select which robot receives movement commands');
            app.TargetDropdown.Layout.Row = 1; app.TargetDropdown.Layout.Column = [1 3];

            labels = {'↑', '↺', '↻', '←', 'STOP', '→', '↓', '⎋', '⏻'};
            cmds = {'FORWARD', 'YAW_LEFT', 'YAW_RIGHT', 'LEFT', 'STOP', ...
                'RIGHT', 'BACKWARD', 'UP', 'DOWN'};
            positions = {[2 2], [2 1], [2 3], [3 1], [3 2], [3 3], [4 2], [4 1], [4 3]};
            for j = 1:9
                app.addCmdButton(gl, labels{j}, cmds{j}, positions{j});
            end

            btnLine = uibutton(gl, 'push', 'Text', 'Formation: Line', ...
                'ButtonPushedFcn', @(~,~) app.setFormation('line'));
            btnLine.Layout.Row = 5; btnLine.Layout.Column = [1 3];
            btnGrid = uibutton(gl, 'push', 'Text', 'Formation: Grid', ...
                'ButtonPushedFcn', @(~,~) app.setFormation('grid'));
            btnGrid.Layout.Row = 6; btnGrid.Layout.Column = [1 3];
            btnReset = uibutton(gl, 'push', 'Text', 'Reset All', ...
                'ButtonPushedFcn', @(~,~) app.resetAll());
            btnReset.Layout.Row = 7; btnReset.Layout.Column = [1 3];
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
                uilabel(app.TelemetryGrid, 'Text', [rows{j}, ':'], ...
                    'FontSize', 10, 'FontWeight', 'bold');
                lbl = uilabel(app.TelemetryGrid, ...
                    'Text', '─', 'FontSize', 10);
                lbl.Layout.Row = j; lbl.Layout.Column = 2;
                app.TelemetryLabels.(flds{j}) = lbl;
            end
        end

        function buildStatusBar(app)
            sbGrid = uigridlayout(app.MainGrid, [1 3], ...
                'RowHeight', {'1x'}, ...
                'ColumnWidth', {'1x', 100, 180}, ...
                'Padding', [0 0 0 0], 'RowSpacing', 0, 'ColumnSpacing', 4);
            sbGrid.Layout.Row = 7; sbGrid.Layout.Column = [1 3];
            app.StatusLabel = uilabel(sbGrid, 'Text', 'Ready', ...
                'FontSize', 13, 'FontWeight', 'bold', 'FontColor', [0.2 0.2 0.2], ...
                'VerticalAlignment', 'center');
            app.StatusLabel.Layout.Row = 1; app.StatusLabel.Layout.Column = 1;
            app.PoolLabel = uilabel(sbGrid, 'Text', '', ...
                'FontSize', 11, 'FontColor', [0.4 0.4 0.4], ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'center');
            app.PoolLabel.Layout.Row = 1; app.PoolLabel.Layout.Column = 2;
            app.FPSLabel = uilabel(sbGrid, 'Text', '', ...
                'FontSize', 11, 'FontColor', [0.5 0.5 0.5], ...
                'HorizontalAlignment', 'right', 'VerticalAlignment', 'center');
            app.FPSLabel.Layout.Row = 1; app.FPSLabel.Layout.Column = 3;
        end

        function tryStartPool(app)
            app.updateStatus('Starting pool...');
            app.PoolLabel.Text = 'Pool: ...';
            app.PoolTimer = timer('ExecutionMode', 'singleShot', 'StartDelay', 0.5, ...
                'TimerFcn', @(~,~) app.doStartPool());
            start(app.PoolTimer);
        end

        function doStartPool(app)
            if ~isvalid(app); return; end
            try
                if ~exist('gcp', 'file')
                    app.PoolAvailable = false;
                    app.PoolLabel.Text = 'Pool: N/A';
                    app.updateStatus('Ready');
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
                    app.PoolLabel.Text = sprintf('Pool: %dw', pool.NumWorkers);
                    app.updateStatus('Ready');
                else
                    app.PoolLabel.Text = 'Pool: ERR';
                    app.updateStatus('Ready');
                end
            catch ME
                app.PoolAvailable = false;
                app.PoolLabel.Text = 'Pool: OFF';
                app.updateStatus('Ready');
            end
        end

        function spawnRobot(app, type)
            n = find(cellfun(@isempty, app.Robots), 1);
            if isempty(n)
                app.updateStatus('All robot slots occupied');
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
            r.Id = sprintf('R%d', app.RobotCounter);

            spawnOffset = [(-0.45)*(n-1) + 0.675, 0, 0];
            r.State(1:2) = r.State(1:2) + spawnOffset(1:2)';

            if isempty(app.SceneVisualizer)
                gx = [-5 5 5 -5]; gy = [-5 -5 5 5]; gz = [0 0 0 0];
                patch(app.SceneAxes, gx, gy, gz, [0.85 0.85 0.85]);
                app.SceneVisualizer = robot.Visualizer(app.SceneAxes);
            end
            app.SceneVisualizer.addRobot(r);
            app.SceneVisualizer.update(r);
            app.Robots{n} = r;
            app.RobotVisible(n) = true;
            app.updateTargetDropdown();

            app.BBoxHandles{n} = [];
            app.BBoxVisible(n) = false;
            app.LegendCheckboxes(n).vis.Value = 1;
            app.LegendCheckboxes(n).vis.Visible = 'on';
            app.LegendCheckboxes(n).vis.Enable = 'on';
            app.toggleVisibility(n);
            app.LegendCheckboxes(n).bbox.Visible = 'on';
            app.LegendCheckboxes(n).bbox.Enable = 'on';
            app.LegendCheckboxes(n).label.Visible = 'on';
            app.LegendCheckboxes(n).label.Text = r.Id;
            c = app.ColorPalette{mod(n-1,4)+1};
            app.LegendCheckboxes(n).label.FontColor = c;
            app.selectRobot(n);
            app.updateStatus(sprintf('SUCCESS: %s spawned at slot %d', r.Id, n));
        end

        function spawnRobotCustom(app, type)
            n = find(cellfun(@isempty, app.Robots), 1);
            if isempty(n)
                app.updateStatus('All robot slots occupied');
                return;
            end
            spawnOffset = [(-0.45)*(n-1) + 0.675, 0, 0];
            params = app.defaultParams(type);
            switch type
                case 'DifferentialDrive'; defaultZ = 0;
                case 'Quadcopter';         defaultZ = 0.5;
                case 'Quadruped'
                    defaultZ = params.geometric.bodyHeight/2 ...
                             + params.kinematic.legLength1 ...
                             + params.kinematic.legLength2;
                case 'Humanoid'
                    defaultZ = params.kinematic.thighLength ...
                             + params.kinematic.shinLength;
            end

            dlg = uifigure('Name', 'Custom Spawn', ...
                'Position', [600 400 380 300], 'Resize', 'off');
            gl = uigridlayout(dlg, [9 2], ...
                'RowHeight', repmat({28}, 1, 9), ...
                'ColumnWidth', {110, '1x'}, ...
                'Padding', [12 12 12 12], 'RowSpacing', 6);

            fields = {'Name:', 'X:', 'Y:', 'Z:', 'Roll (deg):', 'Pitch (deg):', 'Yaw (deg):'};
            defaults = {sprintf('R%d', app.RobotCounter+1), ...
                sprintf('%.3f', spawnOffset(1)), ...
                sprintf('%.3f', spawnOffset(2)), ...
                sprintf('%.3f', defaultZ), ...
                '0', '0', '0'};
            edits = gobjects(7, 1);
            for i = 1:7
                uilabel(gl, 'Text', fields{i}, 'FontSize', 11, ...
                    'FontWeight', 'bold', 'HorizontalAlignment', 'right');
                edits(i) = uieditfield(gl, 'text', 'Value', defaults{i});
            end

            btnGl = uigridlayout(gl, [1 2], 'ColumnWidth', {'1x', '1x'}, 'Padding', [0 0 0 0]);
            btnGl.Layout.Row = 9; btnGl.Layout.Column = [1 2];
            okBtn = uibutton(btnGl, 'push', 'Text', 'Spawn', ...
                'ButtonPushedFcn', @(~,~) spawnCb());
            uibutton(btnGl, 'push', 'Text', 'Cancel', ...
                'ButtonPushedFcn', @(~,~) delete(dlg));

            function spawnCb()
                name = strtrim(edits(1).Value);
                vals = zeros(6,1);
                for i = 1:6
                    vals(i) = str2double(edits(i+1).Value);
                end
                if any(isnan(vals))
                    app.updateStatus('Invalid number in custom spawn fields');
                    return;
                end
                pos = vals(1:3);
                rpy = deg2rad(vals(4:6));
                app.RobotCounter = app.RobotCounter + 1;
                switch type
                    case 'DifferentialDrive'; r = robot.DifferentialDrive(params);
                    case 'Quadcopter';         r = robot.Quadcopter(params);
                    case 'Quadruped';          r = robot.Quadruped(params);
                    case 'Humanoid';           r = robot.Humanoid(params);
                end
                r.Id = name;
                r.State(1:3) = pos;
                r.State(4:7) = robot.Utils.rpyToQuat(rpy(1), rpy(2), rpy(3));

                if isempty(app.SceneVisualizer)
                    gx = [-5 5 5 -5]; gy = [-5 -5 5 5]; gz = [0 0 0 0];
                    patch(app.SceneAxes, gx, gy, gz, [0.85 0.85 0.85]);
                    app.SceneVisualizer = robot.Visualizer(app.SceneAxes);
                end
                app.SceneVisualizer.addRobot(r);
                app.SceneVisualizer.update(r);
                app.Robots{n} = r;
                app.RobotVisible(n) = true;
                app.updateTargetDropdown();

                app.BBoxHandles{n} = [];
                app.BBoxVisible(n) = false;
                app.LegendCheckboxes(n).vis.Value = 1;
                app.LegendCheckboxes(n).vis.Visible = 'on';
                app.LegendCheckboxes(n).vis.Enable = 'on';
                app.toggleVisibility(n);
                app.LegendCheckboxes(n).bbox.Visible = 'on';
                app.LegendCheckboxes(n).bbox.Enable = 'on';
                app.LegendCheckboxes(n).label.Visible = 'on';
                app.LegendCheckboxes(n).label.Text = r.Id;
                c = app.ColorPalette{mod(n-1,4)+1};
                app.LegendCheckboxes(n).label.FontColor = c;
                app.selectRobot(n);
                app.updateStatus(sprintf('SUCCESS: %s spawned at slot %d', r.Id, n));
                delete(dlg);
            end
        end

        function removeRobot(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            r = app.Robots{idx};
            if isprop(r, 'GraphicsTransform') && isvalid(r.GraphicsTransform)
                delete(r.GraphicsTransform.Children);
                delete(r.GraphicsTransform);
            end
            app.Robots{idx} = [];
            app.BBoxHandles{idx} = [];
            app.RobotVisible(idx) = false;
            app.BBoxVisible(idx) = false;
            app.LegendCheckboxes(idx).vis.Value = 0;
            app.LegendCheckboxes(idx).vis.Visible = 'off';
            app.LegendCheckboxes(idx).vis.Enable = 'off';
            app.LegendCheckboxes(idx).bbox.Value = 0;
            app.LegendCheckboxes(idx).bbox.Visible = 'off';
            app.LegendCheckboxes(idx).bbox.Enable = 'off';
            app.LegendCheckboxes(idx).label.Visible = 'off';
            app.LegendCheckboxes(idx).label.Text = '─';
            app.LegendCheckboxes(idx).label.FontColor = [0.5 0.5 0.5];
            if app.SelectedIdx == idx
                app.SelectedIdx = 0;
            end
            app.updateTargetDropdown();
            app.updateStatus(sprintf('Removed %s', r.Id));
        end

        function updateTargetDropdown(app)
            items = {'ALL'};
            for i = 1:numel(app.Robots)
                if ~isempty(app.Robots{i})
                    items{end+1} = sprintf('R%d', i);
                end
            end
            app.TargetDropdown.Items = items;
            if ~ismember(app.TargetDropdown.Value, items)
                app.TargetDropdown.Value = 'ALL';
            end
        end

        function selectRobot(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            for i = 1:numel(app.Robots)
                if isfield(app.LegendCheckboxes, 'label') && isvalid(app.LegendCheckboxes(i).label)
                    app.LegendCheckboxes(i).label.FontWeight = 'normal';
                end
            end
            app.SelectedIdx = idx;
            if isfield(app.LegendCheckboxes, 'label') && isvalid(app.LegendCheckboxes(idx).label)
                app.LegendCheckboxes(idx).label.FontWeight = 'bold';
            end
            app.updateTelemetry();
        end

        function startSimulation(app)
            app.Running = true;
            app.SimTimer = timer('ExecutionMode', 'fixedSpacing', ...
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
            drawnow('limitrate');
            try
                tStart = tic;
                active = find(~cellfun(@isempty, app.Robots));
                baseSteps = ceil(app.RenderDt / app.PhysicsDt);
                nSteps = max(1, round(baseSteps / max(1, 0.25 * numel(active) + 0.75)));
                if app.ScriptMode && ~isempty(active)
                    app.playbackStep(nSteps);
                end
                for k = 1:nSteps
                    for i = active(:)'
                        r = app.Robots{i};
                        if ~app.ScriptMode
                            dir = app.DesiredDirection;
                            amt = app.DesiredAmount;
                            target = app.TargetDropdown.Value;
                            if strcmp(target, 'ALL') || ...
                               strcmp(target, sprintf('R%d', i))
                                r.move(dir, amt);
                            end
                        end
                        r.step(app.SimTime, app.PhysicsDt);
                    end
                    app.SimTime = app.SimTime + app.PhysicsDt;
                end
                for i = active(:)'
                    if app.RobotVisible(i) && ~isempty(app.SceneVisualizer)
                        r = app.Robots{i};
                        if isprop(r, 'GraphicsTransform') && isvalid(r.GraphicsTransform)
                            app.SceneVisualizer.update(r);
                        end
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
                app.FPSLabel.Text = sprintf('%.0f FPS  |  Sim %.1fs', fps, app.SimTime);
            catch ME
                app.updateStatus(sprintf('simStep error: %s', ME.message));
            end
        end

        function drawBoundingBox(app, idx)
            if idx < 1 || idx > numel(app.Robots) || isempty(app.Robots{idx})
                return;
            end
            r = app.Robots{idx};
            if ~isvalid(app.SceneAxes); return; end
            % Update verts from current robot state
            [obbVerts, obbEdges] = robot.Collision.buildOBB(r);
            hVec = app.BBoxHandles{idx};
            if isempty(hVec) || ~all(isvalid(hVec))
                % First time — create line handles
                hold(app.SceneAxes, 'on');
                c = app.ColorPalette{mod(idx-1,4)+1} * 1.5;
                c = min(c, 1);
                hVec = gobjects(size(obbEdges,1), 1);
                for e = 1:size(obbEdges,1)
                    hVec(e) = plot3(app.SceneAxes, obbVerts(obbEdges(e,:),1), ...
                        obbVerts(obbEdges(e,:),2), ...
                        obbVerts(obbEdges(e,:),3), ...
                        'Color', c, 'LineWidth', 1.5, 'LineStyle', '--');
                end
                app.BBoxHandles{idx} = hVec;
            else
                % Update existing handles with new vertex positions
                for e = 1:size(obbEdges,1)
                    set(hVec(e), 'XData', obbVerts(obbEdges(e,:),1), ...
                                 'YData', obbVerts(obbEdges(e,:),2), ...
                                 'ZData', obbVerts(obbEdges(e,:),3));
                end
            end
        end

        function hideBoundingBox(app, idx)
            if ~isempty(app.BBoxHandles{idx})
                if all(isvalid(app.BBoxHandles{idx}))
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
            if ~isprop(r, 'GraphicsTransform') || ~isvalid(r.GraphicsTransform)
                return;
            end
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

        function sendCommand(app, cmdStr)
            dir = robot.Direction.(cmdStr);
            amt = 1.0;
            if any(strcmp(cmdStr, {'YAW_LEFT', 'YAW_RIGHT'}))
                amt = 0.5;
            elseif strcmp(cmdStr, 'STOP')
                amt = 0;
            end
            app.DesiredDirection = dir;
            app.DesiredAmount = amt;
            target = app.TargetDropdown.Value;
            if strcmp(target, 'ALL')
                for i = find(~cellfun(@isempty, app.Robots))'
                    app.Robots{i}.move(dir, amt);
                end
                app.updateStatus(sprintf('Command %s → ALL robots', cmdStr));
            else
                idx = sscanf(target, 'R%d');
                if idx >= 1 && idx <= numel(app.Robots) && ~isempty(app.Robots{idx})
                    app.Robots{idx}.move(dir, amt);
                    app.selectRobot(idx);
                    app.updateStatus(sprintf('Command %s → %s', cmdStr, target));
                else
                    app.updateStatus(sprintf('Cannot send to %s: no robot', target));
                end
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
            if ~isempty(app.PoolTimer) && isvalid(app.PoolTimer)
                stop(app.PoolTimer);
                delete(app.PoolTimer);
            end
            delete(app.Figure);
        end

        function addLegendCheckbox(app, idx)
            visCb = uicheckbox(app.LegendGrid, 'Value', 0, 'Visible', 'off', ...
                'ValueChangedFcn', @(~,~) app.toggleVisibility(idx));
            visCb.Layout.Row = 1+idx; visCb.Layout.Column = 1;
            bboxCb = uicheckbox(app.LegendGrid, 'Value', 0, 'Visible', 'off', ...
                'ValueChangedFcn', @(~,~) app.toggleBoundingBox(idx));
            bboxCb.Layout.Row = 1+idx; bboxCb.Layout.Column = 2;
            lbl = uilabel(app.LegendGrid, 'Text', '─', 'Visible', 'off', ...
                'FontSize', 10, 'FontColor', [0.5 0.5 0.5]);
            lbl.Layout.Row = 1+idx; lbl.Layout.Column = 3;
            app.LegendCheckboxes(idx).vis = visCb;
            app.LegendCheckboxes(idx).bbox = bboxCb;
            app.LegendCheckboxes(idx).label = lbl;
        end

        function addCmdButton(app, gl, label, command, pos)
            btn = uibutton(gl, 'push', 'Text', label, ...
                'FontSize', 11, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.sendCommand(command));
            btn.Layout.Row = pos(1); btn.Layout.Column = pos(2);
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
