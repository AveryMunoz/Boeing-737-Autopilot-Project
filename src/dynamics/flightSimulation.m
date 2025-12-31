function dXdt = flightSimulation(t, X, params, elevator_cmd, throttle_cmd)
    
    % State vector: [x; h; V_x; V_z; theta; q]
    x = X(1);
    h = X(2);
    V_x = X(3);
    V_z = X(4);
    theta = X(5);
    q = X(6);        % Pitch rate
    
    forces = calculateForces(X, params, elevator_cmd, throttle_cmd);

    % State derivatives
    dxdt = V_x;
    dhdt = V_z;
    dVxdt = forces.F_x / forces.mass;
    dVzdt = forces.F_z / forces.mass;
    dthetadt = q;
    dqdt = forces.M / params.airplane.I_yy;
    
    % NEW: 6 states now (was 4)
    dXdt = [dxdt; dhdt; dVxdt; dVzdt; dthetadt; dqdt];

end