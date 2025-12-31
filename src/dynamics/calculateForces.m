function forces = calculateForces(X, params, elevator_cmd, throttle_cmd)
    % Extract state variables
    x = X(1);
    h = X(2);
    V_x = X(3);
    V_z = X(4);
    theta = X(5);
    q = X(6);

    % Extract parameters
    S = params.airplane.wingSurfArea;
    m = params.airplane.mass;
    Tm = params.airplane.maxThrust;
    rho_sl = params.atm.rho;
    T_sl = params.atm.T_sl;  
    g = params.airplane.gravity;
    
    % Aero coefficients
    CL_poly = params.airplane.CL_poly;      % [a, b, c]
    alpha_stall = params.airplane.alpha_stall;
    CD_0 = params.airplane.CD_0;
    CD_k = params.airplane.CD_k;
    
    % Pitch dynamics5
    I_yy = params.airplane.I_yy;
    Cm_alpha = params.airplane.Cm_alpha;
    Cm_q = params.airplane.Cm_q;
    Cm_elevator = params.airplane.Cm_elevator;
    c_bar = params.airplane.c_bar;
    
    % Atmosphere
    [airDensity, T] = atmosphere_model(h, params.atm);
    
    % Velocity and angles
    V = sqrt(V_x^2 + V_z^2);
    gamma = atan2(V_z, V_x);
    
    % Angle of attack
    alpha = theta - gamma;
    alpha_deg = alpha * 180/pi;
    
    % POLYNOMIAL CL MODEL
    if alpha_deg < alpha_stall
        % Quadratic fit: CL = a*alpha^2 + b*alpha + c
        CL = CL_poly(1)*alpha_deg^2 + CL_poly(2)*alpha_deg + CL_poly(3);
    else
        % Post-stall: use stall CL with reduction factor
        CL_stall = CL_poly(1)*alpha_stall^2 + CL_poly(2)*alpha_stall + CL_poly(3);
        CL = CL_stall * 0.6;  % 60% of stall CL
    end
    
    % CD from parabolic polar
    CD = CD_0 + CD_k * CL^2;
    
    % Forces
    Lift = 0.5 * airDensity * S * V^2 * CL;
    Drag = 0.5 * airDensity * S * V^2 * CD;

    thrust_lapse = (airDensity/rho_sl) * (T_sl/T)^1.5;
    Thrust = throttle_cmd * Tm * thrust_lapse;
    %Thrust = throttle_cmd * Tm * (airDensity / rho_sl);

    Weight = m * g;
    
    % Force components in x-z plane
    T_x = Thrust * cos(theta);
    T_z = Thrust * sin(theta);
    
    D_x = -Drag * cos(gamma);
    D_z = -Drag * sin(gamma);
    
    L_x = -Lift * sin(gamma);
    L_z = Lift * cos(gamma);
    
    W_x = 0;
    W_z = -Weight;
    
    % Net forces
    F_x = T_x + D_x + L_x + W_x;
    F_z = T_z + D_z + L_z + W_z;
    
    % Pitching moment
    q_hat = q * c_bar / (2 * V);
    Cm_total = Cm_alpha * alpha + Cm_q * q_hat + Cm_elevator * elevator_cmd;
    M = 0.5 * airDensity * V^2 * S * c_bar * Cm_total;

    % Output structure
    forces.F_x   = F_x;
    forces.F_z   = F_z;
    forces.M     = M;
    forces.Lift   = Lift;
    forces.Drag   = Drag;
    forces.Thrust = Thrust;
    forces.CL     = CL;
    forces.CD     = CD;
    forces.alpha  = alpha;
    forces.gamma  = gamma;
    forces.mass = m;
end