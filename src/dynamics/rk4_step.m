
function X_next = rk4_step(derivFunc, X, dt, params, elevator_cmd, throttle_cmd)
% RK4 numerical integration step

    % k1 = f(t, X)
    k1 = derivFunc(0, X, params, elevator_cmd, throttle_cmd);

    % k2 = f(t + dt/2, X + dt/2*k1)
    k2 = derivFunc(0, X + 0.5*dt*k1, params, elevator_cmd, throttle_cmd);

    % k3 = f(t + dt/2, X + dt/2*k2)
    k3 = derivFunc(0, X + 0.5*dt*k2, params, elevator_cmd, throttle_cmd);

    % k4 = f(t + dt, X + dt*k3)
    k4 = derivFunc(0, X + dt*k3, params, elevator_cmd, throttle_cmd);

    % Combine them to get next state
    X_next = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

end
