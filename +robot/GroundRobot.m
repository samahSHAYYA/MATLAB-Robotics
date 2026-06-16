classdef GroundRobot < robot.Robot
    %GROUNDROBOT  Intermediate base for ground robots.
    %   Provides a variable-dimension control vector and RK4 step().
    %   Default control dimension is 2 (DifferentialDrive); override
    %   getControlDim() for more (Quadruped uses 6).

    methods
        function obj = GroundRobot()
            %GROUNDROBOT  Construct with default 2-element zero control.
            obj@robot.Robot();
            obj.Control = zeros(2, 1);
            obj.InitialControl = obj.Control;
        end

        function step(obj, t, dt)
            %STEP  Integrate dynamics one step via RK4.
            %   Reads obj.Control, adapts dimension via getControlDim().
            %   Inputs: t  - current time (s), dt - step size (s)
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
        function n = getControlDim(~)
            %GETCONTROLDIM  Control vector dimension (override for n ≠ 2).
            n = 2;
        end
    end
end
