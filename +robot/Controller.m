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
    %     escape    → close

    properties
        Figure      
        Robot       
        Visualizer  
        Running     (1,1) logical
        PhysicsDt   (1,1) double = 0.005
        RenderDt    (1,1) double = 0.02
        LastKey     char
    end

    properties (Access = private)
        DesiredDirection robot.Direction = robot.Direction.STOP
        DesiredAmount    (1,1) double = 0
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
