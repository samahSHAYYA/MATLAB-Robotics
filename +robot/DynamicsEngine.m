classdef (Sealed) DynamicsEngine
    %DYNAMICSENGINE  Static RK4 integrator for 13-element rigid-body state.
    %   State layout: [pos(3); quat(4); vel(3); omega(3)].
    methods (Static)
        function stateOut = rk4Step(dynFun, t, state, control, dt)
            %RK4STEP  Single fourth-order Runge-Kutta integration step.
            %   s_next = robot.DynamicsEngine.rk4Step(f, t, s, u, dt)
            %   Inputs:  dynFun  - @(t,s,u) → dstate (13x1)
            %            t       - current time (s)
            %            state   - current state (13x1)
            %            control - control input vector
            %            dt      - time step (s)
            %   Outputs: stateOut - integrated state (13x1)
            k1 = dynFun(t, state, control);
            k2 = dynFun(t + dt/2, state + (dt/2)*k1, control);
            k3 = dynFun(t + dt/2, state + (dt/2)*k2, control);
            k4 = dynFun(t + dt, state + dt*k3, control);
            stateOut = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        end

        function states = integrate(dynFun, tSpan, state0, control, dt)
            %INTEGRATE  Batch RK4 integration over a time span.
            %   states = robot.DynamicsEngine.integrate(f, [t0 tf], s0, u, dt)
            %   Inputs:  dynFun  - @(t,s,u) → dstate
            %            tSpan   - [t0, tf] start/end times
            %            state0  - initial state (13x1)
            %            control - control vector (constant over span)
            %            dt      - time step (s)
            %   Outputs: states  - N×13 matrix, each row is a state snapshot
            t0 = tSpan(1);
            tf = tSpan(2);
            N = ceil((tf - t0) / dt) + 1;
            states = zeros(N, 13);
            states(1, :) = state0';
            state = state0;
            t = t0;
            for k = 2:N
                state = robot.DynamicsEngine.rk4Step(dynFun, t, state, control, dt);
                states(k, :) = state';
                t = t + dt;
            end
        end
    end
end
