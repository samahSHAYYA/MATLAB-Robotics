function test_rk4_step
% TEST_RK4_STEP Verify RK4 integration with a simple harmonic oscillator
% The SHO has known solution: position = cos(t), velocity = -sin(t)

    dt = 0.01;
    t = 0;

    % Simple harmonic oscillator: dstate = [v; -x]
    dynFun = @(t, s, ~) [s(2); -s(1)];

    state = [1; 0]; % x=1, v=0
    for i = 1:100
        state = rk4Step(dynFun, t, state, [], dt);
        t = t + dt;
    end

    % After 100 steps (t=1), expected: x ≈ cos(1) = 0.5403, v ≈ -sin(1) = -0.8415
    expected = [cos(1); -sin(1)];
    error = norm(state - expected);

    fprintf('RK4 SHO test:\n');
    fprintf('  state = [%.6f, %.6f]\n', state(1), state(2));
    fprintf('  expected = [%.6f, %.6f]\n', expected(1), expected(2));
    fprintf('  error = %.2e\n', error);

    if error < 1e-4
        fprintf('  [PASS] error within tolerance.\n');
    else
        fprintf('  [FAIL] error too large.\n');
    end
end

function nextState = rk4Step(dynFun, t, state, control, dt)
    k1 = dynFun(t, state, control);
    k2 = dynFun(t + dt/2, state + dt/2*k1, control);
    k3 = dynFun(t + dt/2, state + dt/2*k2, control);
    k4 = dynFun(t + dt, state + dt*k3, control);
    nextState = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
