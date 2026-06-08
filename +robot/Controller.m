classdef Controller < handle
    properties
        Figure      (1,1) matlab.ui.Figure
        Robot       (1,1) robot.Robot
        Visualizer  (1,1) robot.Visualizer
        Running     (1,1) logical
        PhysicsDt   (1,1) double = 0.005
        RenderDt    (1,1) double = 0.02
        LastKey     char
    end

    methods
        function obj = Controller(fig, robot, visualizer)
            obj.Figure = fig;
            obj.Robot = robot;
            obj.Visualizer = visualizer;
            obj.Running = true;

            fig.WindowKeyPressFcn = @obj.onKeyPress;
            fig.CloseRequestFcn = @obj.onClose;
        end

        function run(obj)
            t = 0;
            while obj.Running && ishandle(obj.Figure)
                tic;
                for i = 1:ceil(obj.RenderDt / obj.PhysicsDt)
                    obj.Robot.step(t, obj.PhysicsDt);
                    t = t + obj.PhysicsDt;
                end
                obj.Visualizer.update(obj.Robot);
                drawnow limitrate;
                elapsed = toc;
                pause(max(0, obj.RenderDt - elapsed));
            end
        end

        function setCommand(obj, direction, amount)
            obj.Robot.move(direction, amount);
        end
    end

    methods (Access = private)
        function onKeyPress(obj, ~, evt)
            switch evt.Key
                case 'uparrow'
                    obj.Robot.move(robot.Direction.FORWARD, 1.0);
                case 'downarrow'
                    obj.Robot.move(robot.Direction.BACKWARD, 1.0);
                case 'leftarrow'
                    obj.Robot.move(robot.Direction.YAW_LEFT, 0.5);
                case 'rightarrow'
                    obj.Robot.move(robot.Direction.YAW_RIGHT, 0.5);
                case 'w'
                    obj.Robot.move(robot.Direction.UP, 1.0);
                case 's'
                    obj.Robot.move(robot.Direction.DOWN, 1.0);
                case 'a'
                    obj.Robot.move(robot.Direction.ROLL_LEFT, 0.5);
                case 'd'
                    obj.Robot.move(robot.Direction.ROLL_RIGHT, 0.5);
                case 'q'
                    obj.Robot.move(robot.Direction.PITCH_UP, 0.5);
                case 'e'
                    obj.Robot.move(robot.Direction.PITCH_DOWN, 0.5);
                case 'space'
                    obj.Robot.move(robot.Direction.STOP, 0);
                case 'r'
                    obj.Robot.reset();
                case 'escape'
                    obj.onClose();
            end
            obj.LastKey = evt.Key;
        end

        function onClose(obj, ~, ~)
            obj.Running = false;
            delete(obj.Figure);
        end
    end
end
