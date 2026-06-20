classdef Controller < handle
    %CONTROLLER  Interactive keyboard control for robot demos.
    %   Manages the render loop (drawnow → move → step × N → update),
    %   captures keyboard input via WindowKeyPressFcn/WindowKeyReleaseFcn
    %   for momentary (press-to-move, release-to-stop) control.
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
    %     escape    → close

    properties
        Figure      
        Robot       
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
    end

    methods
        function obj = Controller(fig, robot, visualizer)
            %CONTROLLER  Attach keyboard callbacks to figure.
            %   Inputs: fig        - matlab.ui.Figure handle
            %           robot      - robot.Robot subclass instance
            %           visualizer - robot.Visualizer instance
            %   Registers WindowKeyPressFcn, WindowKeyReleaseFcn, and
            %   CloseRequestFcn on the figure.
            validateattributes(fig, {'matlab.ui.Figure'}, {'scalar'});
            obj.Figure = fig;
            obj.Robot = robot;
            obj.Visualizer = visualizer;
            obj.Running = true;

            fig.WindowKeyPressFcn = @obj.onKeyPress;
            fig.WindowKeyReleaseFcn = @obj.onKeyRelease;
            fig.CloseRequestFcn = @obj.onClose;

            obj.HudActive = false;
            obj.CameraModeIdx = 1;
            obj.Visualizer.CameraMode = "free";
        end

        function run(obj)
            %RUN  Main simulation loop.
            %   Flushes stale events, then loops:
            %     drawnow         → process keyboard events
            %     robot.move()    → apply direction/amount
            %     robot.step() ×N → RK4 physics (5 ms sub-steps)
            %     visualizer.update()
            %   Target render rate: 1/RenderDt = 50 fps.
            drawnow;
            obj.DesiredDirection = robot.Direction.STOP;
            obj.DesiredAmount = 0;
            t = 0;
            while obj.Running && ishandle(obj.Figure)
                drawnow;
                if ~obj.Running; break; end
                obj.Robot.move(obj.DesiredDirection, obj.DesiredAmount);
                tic;
                for i = 1:ceil(obj.RenderDt / obj.PhysicsDt)
                    try
                        obj.Robot.step(t, obj.PhysicsDt);
                    catch ME
                        fprintf('Error in step: %s\n', ME.message);
                        obj.Running = false;
                        break;
                    end
                    t = t + obj.PhysicsDt;
                end
                if ~obj.Running; break; end
                try
                    obj.Visualizer.update(obj.Robot);
                catch ME
                    fprintf('Error in update: %s\n', ME.message);
                    obj.Running = false;
                    break;
                end
                switch obj.PathMode
                    case "record"
                        s = obj.Robot.State;
                        if norm(s(8:10)) > 0.02
                            obj.RecordedPath(end+1,:) = [t, s(1), s(2), s(3)];
                        end
                    case "replay"
                        obj.runReplayStep();
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
            %SETCOMMAND  Directly apply a move command (for programmatic use).
            %   Inputs: direction - robot.Direction enum
            %           amount    - [0,1] scalar
            obj.Robot.move(direction, amount);
        end

        function initHud(obj)
            f = obj.Figure;
            pos = [0.75, 0.85, 0.22, 0.12];
            obj.HudHandles.Speed = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.00, 0, 0], ...
                'String', 'Speed: 0.00 m/s', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Altitude = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.04, 0, 0], ...
                'String', 'Alt: 0.00 m', ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'none', ...
                'Color', 'white', 'HorizontalAlignment', 'left', ...
                'Visible', 'off');
            obj.HudHandles.Battery = annotation(f, 'textbox', ...
                'Position', pos + [0, -0.08, 0, 0], ...
                'String', 'Batt: 85%', ...
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
            s = obj.Robot.State;
            vel = norm(s(8:10));
            alt = s(3);
            gaitStr = "";
            if isprop(obj.Robot, 'GaitEnabled') && obj.Robot.GaitEnabled
                gaitStr = " gait";
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
                set(obj.HudHandles.Mode, 'String', ['Mode: ' cam '  Path: ' pm gaitStr]);
            end
        end

        function startReplay(obj)
            if size(obj.RecordedPath, 1) < 2
                obj.PathMode = "manual";
                fprintf('Path too short for replay.\n');
                return;
            end
            obj.Robot.reset();
            obj.ReplayIdx = 1;
            obj.ReplayTime = 0;
            startPos = obj.RecordedPath(1, 2:4);
            obj.Robot.setState([startPos(1); startPos(2); startPos(3); 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
            obj.Visualizer.update(obj.Robot);
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
                obj.Robot.reset();
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
            obj.Robot.setState([pos(1); pos(2); pos(3); 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
        end

        function updateCamera(obj, t)
            ax = obj.Visualizer.AxesHandle;
            pos = obj.Robot.State(1:3);
            R = robot.Utils.quatToRotmx(obj.Robot.State(4:7));
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
        function onKeyPress(obj, ~, evt)
            %ONKEYPRESS  Map key event to Direction + amount.
            %   Called by WindowKeyPressFcn during drawnow.
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
                    if isa(obj.Robot, 'robot.Quadruped') || ...
                       isa(obj.Robot, 'robot.Humanoid')
                        obj.Robot.toggleGait();
                    end
                case 'space'
                    obj.DesiredDirection = robot.Direction.STOP;
                    obj.DesiredAmount = 0;
                case 'r'
                    obj.Robot.reset();
                    obj.DesiredDirection = robot.Direction.STOP;
                    obj.DesiredAmount = 0;
                case 'l'
                    if isprop(obj.Robot, 'LightsOn')
                        obj.Robot.LightsOn = ~obj.Robot.LightsOn;
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
                    modes = ["manual", "record", "replay"];
                    switch obj.PathMode
                        case "manual"; obj.PathMode = "record";
                        case "record"; obj.PathMode = "replay"; obj.startReplay();
                        case "replay"; obj.PathMode = "manual"; obj.RecordedPath = zeros(0,4);
                    end
                    fprintf('Path mode: %s\n', obj.PathMode);
                case 'escape'
                    obj.onClose();
            end
            obj.LastKey = evt.Key;
        end

        function onKeyRelease(obj, ~, ~)
            %ONKEYRELEASE  Reset to STOP on any key release.
            %   This provides momentary control (move while held).
            obj.DesiredDirection = robot.Direction.STOP;
            obj.DesiredAmount = 0;
        end

        function onClose(obj, ~, ~)
            %ONCLOSE  Stop the loop and delete the figure.
            obj.Running = false;
            delete(obj.Figure);
        end
    end
end
