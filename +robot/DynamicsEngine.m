classdef (Sealed) DynamicsEngine
    methods (Static)
        function stateOut = rk4Step(dynFun, t, state, control, dt)
            k1 = dynFun(t, state, control);
            k2 = dynFun(t + dt/2, state + (dt/2)*k1, control);
            k3 = dynFun(t + dt/2, state + (dt/2)*k2, control);
            k4 = dynFun(t + dt, state + dt*k3, control);
            stateOut = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        end

        function states = integrate(dynFun, tSpan, state0, control, dt)
            t0 = tSpan(1);
            tf = tSpan(2);
            N = ceil((tf - t0) / dt) + 1;
            states = zeros(N, 13);
            states(1, :) = state0';
            state = state0;
            t = t0;
            for k = 2:N
                state = DynamicsEngine.rk4Step(dynFun, t, state, control, dt);
                states(k, :) = state';
                t = t + dt;
            end
        end
    end
end
