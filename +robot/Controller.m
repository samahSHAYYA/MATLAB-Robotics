classdef Controller < handle
    %CONTROLLER  Interactive keyboard control for robot demos.
    %   Manages the render loop (drawnow → move → step × N → update),
    %   captures keyboard input via WindowKeyPressFcn/WindowKeyReleaseFcn
    %   for momentary (press-to-move, release-to-stop) control.
    %
    %   Supports multiple robots in a single scene with active robot
    %   switching via Tab / Shift+Tab / 1-9 keys.
    %
    %   Key bindings:
    %     arrows    → FORWARD/BACKWARD/YAW_LEFT/YAW_RIGHT
    %     w/s       → UP/DOWN (aerial robots)
    %     a/d       → ROLL_LEFT/ROLL_RIGHT (aerial robots)
    %     q/e       → PITCH_UP/PITCH_DOWN (aerial robots)
    %     space     → STOP
    %     r         → RESET
    %     g         → toggle gait (Quadruped, Humanoid)
    %     h         → toggle HUD overlay
    %     c         → cycle camera mode (free → chase → orbit → top)
    %     l         → toggle running lights
    %     p         → cycle path mode (manual → record → replay → manual)
    %     n         → cycle waypoint mode (off → place → navigate → off)
    %     tab       → cycle active robot forward
    %     shift+tab → cycle active robot backward
    %     1-9       → jump to robot N
    %     click     → place waypoint at mouse location (in place mode)
    %     escape    → close

    properties
        Figure      
        Robots              cell
        ActiveIdx   (1,1) double = 1
        Visualizer  
        Running     (1,1) logical
        PhysicsDt   (1,1) double = 0.005
        RenderDt    (1,1) double = 0.02
        LastKey     char
        HudActive   (1,1) logical = false
        HudHandles  (1,1) struct
        CameraModeIdx (1,1) double = 1
    end

    properties (Access = private)
        DesiredDirection robot.Direction = robot.Direction.STOP
        DesiredAmount    (1,1) double = 0
        OrbitAngle       (1,1) double = 0
        HudInitialized   (1,1) logical = false
        PathMode         (1,1) string = "manual"
        RecordedPath     (:,4) double = zeros(0,4)
        ReplayIdx        (1,1) double = 1
        ReplayTime       (1,1) double = 0
        WaypointRadius   (1,1) double = 0.15
    end

    properties
        WaypointMode     (1,1) string = "off"
        Waypoints        (:,3) double = zeros(0,3)
        WaypointTargetIdx (1,1) double = 1
    end

    methods
        function obj = Controller(fig, robots, visualizer)
            %CONTROLLER  Attach keyboard callbacks to figure.
            %   Inputs: fig        - matlab.ui.Figure handle
            %           robots     - robot.Robot instance or cell array of robots
            %           visualizer - robot.Visualizer instance
            %   Registers WindowKeyPressFcn, WindowKeyReleaseFcn, and
            %   CloseRequestFcn on the figure.
            validateattributes(fig, {'matlab.ui.Figure'}, {'scalar'});
            obj.Figure = fig;
            if iscell(robots)
                obj.Robots = robots(:)';
            else
                obj.Robots = {robots};
            end
            obj.Visualizer = visualizer;
            obj.Running = true;
            obj.ActiveIdx = 1;

            fig.WindowKeyPressFcn = @obj.onKeyPress;
            fig.WindowKeyReleaseFcn = @obj.onKeyRelease;
            fig.CloseRequestFcn = @obj.onClose;

            obj.HudActive = false;
            obj.CameraModeIdx = 1;
            obj.Visualizer.CameraMode = "free";
            obj.Visualizer.AxesHandle.ButtonDownFcn = @obj.onAxesClick;
        end

        function r = ActiveRobot(obj)
            r = obj.Robots{obj.ActiveIdx};
        end

        function run(obj)
            %RUN  Main simulation loop.
            %   Flushes stale events, then loops:
            %     drawnow         → process keyboard events
            %     robot.move()    → apply direction/amount to active robot
            %     robot.step() ×N → RK4 physics for ALL robots
            %     visualizer.update() for each robot
            %   Target render rate: 1/RenderDt = 50 fps.
            drawnow;
            for i = 1:length(obj.Robots)
                obj.Robots{i}.move(robot.Direction.STOP, 0);
            end
            obj.DesiredDirection = robot.Direction.STOP;
            obj.DesiredAmount = 0;
            t = 0;
            while obj.Running && ishandle(obj.Figure)
                drawnow;
                if ~obj.Running; break; end
                obj.ActiveRobot.move(obj.DesiredDirection, obj.DesiredAmount);
                tic;
                for i = 1:ceil(obj.RenderDt / obj.PhysicsDt)
                    for j = 1:length(obj.Robots)
                        try
                            obj.Robots{j}.step(t, obj.PhysicsDt);
                        catch ME
                            fprintf('Error in step (%s): %s\n', ...
                                class(obj.Robots{j}), ME.message);
                            obj.Running = false;
                            break;
                        end
                    end
                    t = t + obj.PhysicsDt;
                end
                if ~obj.Running; break; end
                for j = 1:length(obj.Robots)
                    try
                        obj.Visualizer.update(obj.Robots{j});
                    catch ME
                        fprintf('Error in update (%s): %s\n', ...
                            class(obj.Robots{j}), ME.message);
                        obj.Running = false;
                        break;
                    end
                end
                if ~obj.Running; break; end
                switch obj.PathMode
                    case "record"
                        s = obj.ActiveRobot.State;
                        if norm(s(8:10)) > 0.02
                            obj.RecordedPath(end+1,:) = [t, s(1), s(2), s(3)];
                        end
                    case "replay"
                        obj.runReplayStep();
                end
                if strcmp(obj.WaypointMode, "navigate")
                    obj.navigateToWaypoint();
                end
                if obj.HudActive
                    obj.updateHud();
                end
                if ~strcmp(obj.Visualizer.CameraMode, "free")
                    obj.updateCamera(t);
                end
                elapsed = toc;
                pause(max(0, obj.RenderDt - elapsed));
            end
            delete(obj.Figure);
        end

        function setCommand(obj, direction, amount)
            %SETCOMMAND  Directly apply a move command to the active robot.
            obj.ActiveRobot.move(direction, amount);
        end

        function initHud(obj)
            f = obj.Figure;
            pos = [0.75, 0.82, 0.22, 0.15];
            obj.HudHandles.Robot = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.00, 0, 0], ...
                'String', '', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Speed = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.04, 0, 0], ...
                'String', 'Speed: 0.00 m/s', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Altitude = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.08, 0, 0], ...
                'String', 'Alt: 0.00 m', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Mode = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.12, 0, 0], ...
                'String', 'Mode: manual', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Battery = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.16, 0, 0], ...
                'String', 'Batt: 85%', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudInitialized = true;
        end

        function showHud(obj, on)
            fns = fieldnames(obj.HudHandles);
            vis = 'off';
            if on; vis = 'on'; end
            for i = 1:length(fns)
                h = obj.HudHandles.(fns{i});
                if ishandle(h); set(h, 'Visible', vis); end
            end
        end

        function updateHud(obj)
            rbt = obj.ActiveRobot;
            s = rbt.State;
            vel = norm(s(8:10));
            alt = s(3);
            gaitStr = "";
            if isprop(rbt, 'GaitEnabled') && rbt.GaitEnabled
                gaitStr = " gait";
            end
            nRobots = length(obj.Robots);
            label = sprintf('%s [%d/%d]', class(rbt), obj.ActiveIdx, nRobots);
            if ishandle(obj.HudHandles.Robot)
                set(obj.HudHandles.Robot, 'String', label);
            end
            if ishandle(obj.HudHandles.Speed)
                set(obj.HudHandles.Speed, 'String', sprintf('Speed: %.2f m/s', vel));
            end
            if ishandle(obj.HudHandles.Altitude)
                set(obj.HudHandles.Altitude, 'String', sprintf('Alt: %.2f m', alt));
            end
            if ishandle(obj.HudHandles.Mode)
                cam = char(obj.Visualizer.CameraMode);
                pm = char(obj.PathMode);
                wm = char(obj.WaypointMode);
                wpStr = '';
                if ~strcmp(obj.WaypointMode, "off")
                    wpStr = sprintf('  WP: %s[%d/%d]', wm, ...
                        obj.WaypointTargetIdx, size(obj.Waypoints,1));
                end
                set(obj.HudHandles.Mode, 'String', ...
                    ['Mode: ' cam '  Path: ' pm gaitStr wpStr]);
            end
        end

        function startReplay(obj)
            if size(obj.RecordedPath, 1) < 2
                obj.PathMode = "manual";
                fprintf('Path too short for replay.\n');
                return;
            end
            rbt = obj.ActiveRobot;
            rbt.reset();
            obj.ReplayIdx = 1;
            obj.ReplayTime = 0;
            startPos = obj.RecordedPath(1, 2:4);
            rbt.setState([startPos(1); startPos(2); startPos(3); 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
            obj.Visualizer.update(rbt);
        end

        function runReplayStep(obj)
            if obj.ReplayIdx >= size(obj.RecordedPath, 1)
                return;
            end
            obj.ReplayTime = obj.ReplayTime + obj.RenderDt;
            while obj.ReplayIdx < size(obj.RecordedPath, 1) && ...
                  obj.RecordedPath(obj.ReplayIdx+1, 1) < obj.ReplayTime
                obj.ReplayIdx = obj.ReplayIdx + 1;
            end
            if obj.ReplayIdx >= size(obj.RecordedPath, 1)
                obj.ActiveRobot.reset();
                obj.PathMode = "manual";
                fprintf('Replay complete.\n');
                return;
            end
            t0 = obj.RecordedPath(obj.ReplayIdx, 1);
            t1 = obj.RecordedPath(obj.ReplayIdx+1, 1);
            frac = (obj.ReplayTime - t0) / (t1 - t0 + eps);
            frac = max(0, min(1, frac));
            p0 = obj.RecordedPath(obj.ReplayIdx, 2:4);
            p1 = obj.RecordedPath(obj.ReplayIdx+1, 2:4);
            pos = p0 + frac * (p1 - p0);
            obj.ActiveRobot.setState([pos(1); pos(2); pos(3); 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
        end

        function navigateToWaypoint(obj)
            if size(obj.Waypoints, 1) == 0
                obj.WaypointMode = "off";
                fprintf('No waypoints to navigate.\n');
                return;
            end
            if obj.WaypointTargetIdx > size(obj.Waypoints, 1)
                obj.WaypointMode = "off";
                obj.Visualizer.clearWaypoints();
                obj.Waypoints = zeros(0,3);
                obj.WaypointTargetIdx = 1;
                fprintf('Navigation complete.\n');
                return;
            end
            rbt = obj.ActiveRobot;
            pos = rbt.State(1:3);
            target = obj.Waypoints(obj.WaypointTargetIdx, :);
            delta = target - pos';
            dist = norm(delta);
            if dist < obj.WaypointRadius
                obj.WaypointTargetIdx = obj.WaypointTargetIdx + 1;
                obj.Visualizer.highlightWaypoint(obj.WaypointTargetIdx);
                if obj.WaypointTargetIdx > size(obj.Waypoints, 1)
                    obj.DesiredDirection = robot.Direction.STOP;
                    obj.DesiredAmount = 0;
                end
                return;
            end
            yaw = atan2(delta(2), delta(1));
            R = robot.Utils.quatToRotmx(rbt.State(4:7));
            currentYaw = atan2(R(2,1), R(1,1));
            yawErr = mod(yaw - currentYaw + pi, 2*pi) - pi;
            isAerial = isa(rbt, 'robot.Quadcopter');
            if abs(yawErr) > 0.2
                if yawErr > 0
                    obj.DesiredDirection = robot.Direction.YAW_LEFT;
                    obj.DesiredAmount = min(1.0, abs(yawErr) / pi);
                else
                    obj.DesiredDirection = robot.Direction.YAW_RIGHT;
                    obj.DesiredAmount = min(1.0, abs(yawErr) / pi);
                end
            else
                obj.DesiredDirection = robot.Direction.FORWARD;
                obj.DesiredAmount = min(1.0, dist / 0.5);
                if isAerial && delta(3) > 0.1
                    obj.DesiredDirection = robot.Direction.UP;
                    obj.DesiredAmount = min(1.0, delta(3) / 0.5);
                elseif isAerial && delta(3) < -0.1
                    obj.DesiredDirection = robot.Direction.DOWN;
                    obj.DesiredAmount = min(1.0, abs(delta(3)) / 0.5);
                end
            end
            rbt.move(obj.DesiredDirection, obj.DesiredAmount);
        end

        function updateCamera(obj, ~)
            ax = obj.Visualizer.AxesHandle;
            rbt = obj.ActiveRobot;
            pos = rbt.State(1:3);
            R = robot.Utils.quatToRotmx(rbt.State(4:7));
            switch obj.Visualizer.CameraMode
                case "chase"
                    offset = R * [0; -1.5; 0.5];
                    campos(ax, pos + offset);
                    camtarget(ax, pos);
                case "orbit"
                    obj.OrbitAngle = obj.OrbitAngle + 0.02;
                    r = 2.0;
                    campos(ax, pos + [r*cos(obj.OrbitAngle); r*sin(obj.OrbitAngle); 0.8]);
                    camtarget(ax, pos);
                case "top"
                    campos(ax, pos + [0; 0; 2.5]);
                    camtarget(ax, pos);
                    camup(ax, [0; 1; 0]);
            end
        end
    end

    methods (Access = private)
        function switchRobot(obj, idx)
            n = length(obj.Robots);
            obj.ActiveIdx = max(1, min(n, idx));
            obj.DesiredDirection = robot.Direction.STOP;
            obj.DesiredAmount = 0;
            fprintf('Active: %s [%d/%d]\n', ...
                class(obj.Robots{obj.ActiveIdx}), obj.ActiveIdx, n);
        end

        function onKeyPress(obj, ~, evt)
            %ONKEYPRESS  Map key event to Direction + amount.
            %   Called by WindowKeyPressFcn during drawnow.
            rbt = obj.ActiveRobot;
            switch evt.Key
                case 'uparrow'
                    obj.DesiredDirection = robot.Direction.FORWARD;
                    obj.DesiredAmount = 1.0;
                case 'downarrow'
                    obj.DesiredDirection = robot.Direction.BACKWARD;
                    obj.DesiredAmount = 1.0;
                case 'leftarrow'
                    obj.DesiredDirection = robot.Direction.YAW_LEFT;
                    obj.DesiredAmount = 0.5;
                case 'rightarrow'
                    obj.DesiredDirection = robot.Direction.YAW_RIGHT;
                    obj.DesiredAmount = 0.5;
                case 'w'
                    obj.DesiredDirection = robot.Direction.UP;
                    obj.DesiredAmount = 1.0;
                case 's'
                    obj.DesiredDirection = robot.Direction.DOWN;
                    obj.DesiredAmount = 1.0;
                case 'a'
                    obj.DesiredDirection = robot.Direction.ROLL_LEFT;
                    obj.DesiredAmount = 0.5;
                case 'd'
                    obj.DesiredDirection = robot.Direction.ROLL_RIGHT;
                    obj.DesiredAmount = 0.5;
                case 'q'
                    obj.DesiredDirection = robot.Direction.PITCH_UP;
                    obj.DesiredAmount = 0.5;
                case 'e'
                    obj.DesiredDirection = robot.Direction.PITCH_DOWN;
                    obj.DesiredAmount = 0.5;
                case 'g'
                    if isa(rbt, 'robot.Quadruped') || ...
                       isa(rbt, 'robot.Humanoid')
                        rbt.toggleGait();
                    end
                case 'space'
                    obj.DesiredDirection = robot.Direction.STOP;
                    obj.DesiredAmount = 0;
                case 'r'
                    rbt.reset();
                    obj.DesiredDirection = robot.Direction.STOP;
                    obj.DesiredAmount = 0;
                case 'l'
                    if isprop(rbt, 'LightsOn')
                        rbt.LightsOn = ~rbt.LightsOn;
                    end
                case 'h'
                    obj.HudActive = ~obj.HudActive;
                    if obj.HudActive && ~obj.HudInitialized
                        obj.initHud();
                    end
                    if obj.HudActive
                        obj.showHud(true);
                    else
                        obj.showHud(false);
                    end
                case 'c'
                    modes = ["free", "chase", "orbit", "top"];
                    obj.CameraModeIdx = mod(obj.CameraModeIdx, length(modes)) + 1;
                    obj.Visualizer.CameraMode = modes(obj.CameraModeIdx);
                    if strcmp(obj.Visualizer.CameraMode, "free")
                        camva(obj.Visualizer.AxesHandle, 'auto');
                        camproj(obj.Visualizer.AxesHandle, 'perspective');
                    end
                case 'p'
                    switch obj.PathMode
                        case "manual"; obj.PathMode = "record";
                        case "record"; obj.PathMode = "replay"; obj.startReplay();
                        case "replay"; obj.PathMode = "manual"; obj.RecordedPath = zeros(0,4);
                    end
                    fprintf('Path mode: %s\n', obj.PathMode);
                case 'n'
                    switch obj.WaypointMode
                        case "off"
                            obj.WaypointMode = "place";
                            fprintf('Waypoint mode: place — click on scene to place waypoints.\n');
                        case "place"
                            if size(obj.Waypoints, 1) < 2
                                fprintf('Place at least 2 waypoints first.\n');
                                return;
                            end
                            obj.WaypointMode = "navigate";
                            obj.WaypointTargetIdx = 1;
                            obj.Visualizer.highlightWaypoint(1);
                            fprintf('Waypoint mode: navigate — robot moving through waypoints.\n');
                        case "navigate"
                            obj.WaypointMode = "off";
                            obj.Visualizer.clearWaypoints();
                            obj.Waypoints = zeros(0,3);
                            obj.WaypointTargetIdx = 1;
                            fprintf('Waypoint mode: off. Waypoints cleared.\n');
                    end
                case 'tab'
                    n = length(obj.Robots);
                    if n < 2; return; end
                    if ismember('shift', evt.Modifier)
                        obj.switchRobot(mod(obj.ActiveIdx - 2, n) + 1);
                    else
                        obj.switchRobot(mod(obj.ActiveIdx, n) + 1);
                    end
                case {'1','2','3','4','5','6','7','8','9'}
                    n = length(obj.Robots);
                    idx = str2double(evt.Key);
                    if idx <= n
                        obj.switchRobot(idx);
                    end
                case 'escape'
                    obj.onClose();
            end
            obj.LastKey = evt.Key;
        end

        function onKeyRelease(obj, ~, ~)
            %ONKEYRELEASE  Reset to STOP on any key release.
            obj.DesiredDirection = robot.Direction.STOP;
            obj.DesiredAmount = 0;
        end

        function onClose(obj, ~, ~)
            obj.Running = false;
            delete(obj.Figure);
        end

        function onAxesClick(obj, src, ~)
            if ~strcmp(obj.WaypointMode, "place"); return; end
            rbt = obj.ActiveRobot;
            cp = get(src, 'CurrentPoint');
            pos = cp(1, 1:3);
            if isa(rbt, 'robot.DifferentialDrive')
                pos(3) = 0;
            end
            idx = size(obj.Waypoints, 1) + 1;
            obj.Waypoints(idx, :) = pos;
            obj.Visualizer.addWaypointMarker(pos, idx);
            fprintf('Waypoint %d placed at (%.2f, %.2f, %.2f)\n', ...
                idx, pos(1), pos(2), pos(3));
        end
    end
end
