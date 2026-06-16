classdef AerialRobot < robot.Robot
    %AERIALROBOT  Intermediate base for aerial robots (e.g. Quadcopter).
    %   Provides a 4-channel control vector and an RK4-integrating step().

    methods
        function obj = AerialRobot()
            %AERIALROBOT  Construct with 4-element zero control.
            obj@robot.Robot();
            obj.Control = zeros(4, 1);
            obj.InitialControl = obj.Control;
        end

        function step(obj, t, dt)
            %STEP  Integrate dynamics one step via RK4.
            %   Reads obj.Control and passes it to computeDynamics().
            %   Inputs: t  - current time (s), dt - step size (s)
            arguments
                obj
                t (1,1) double
                dt (1,1) double
            end
            if isempty(obj.Control)
                u = zeros(4, 1);
            else
                u = obj.Control;
            end
            dynFun = @(t, s, u) obj.computeDynamics(t, s, u);
            s = robot.DynamicsEngine.rk4Step(dynFun, t, obj.State, u, dt);
            obj.setState(s);
        end

        function hover(obj)
            %HOVER  Zero out the control vector (resets to no thrust).
            arguments
                obj
            end
            obj.Control = zeros(4, 1);
        end
    end
end
