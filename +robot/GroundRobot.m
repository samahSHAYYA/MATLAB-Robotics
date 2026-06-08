classdef GroundRobot < robot.Robot

    methods
        function obj = GroundRobot(params)
            arguments
                params (1,1) struct
            end
            obj@robot.Robot(params);
            obj.Control = zeros(2, 1);
        end

        function step(obj, t, dt)
            arguments
                obj
                t (1,1) double
                dt (1,1) double
            end
            if isempty(obj.Control)
                u = zeros(obj.getControlDim(), 1);
            else
                u = obj.Control;
            end
            dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
            s = robot.DynamicsEngine.rk4Step(dynFun, t, obj.State, u, dt);
            obj.setState(s);
        end
    end

    methods (Access = protected)
        function n = getControlDim(obj)
            n = 2;
        end
    end
end
