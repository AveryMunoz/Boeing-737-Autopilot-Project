function [A, B, f0] = linearizeAircraft(X_trim, u_trim, params)
% LINEARIZEAIRCRAFT  Numerically linearize flight dynamics around a trim point.
%
% Must be done in order to properly utilize PID controllers on a linear
% system in comparison to a nonlinear system. Also helps with testing
% linearization and attaining trim points (equilibrium points where the
% acceleration is 0)
%
%   X_trim : 6x1 state vector at trim [x; h; Vx; Vz; theta; q]
%   u_trim : 2x1 input vector at trim [elevator_cmd; throttle_cmd]
%   params : parameters struct (same as used in flightSimulation)
%
%   Returns:
%     A  : nxn state matrix (continuous time)
%     B  : nxm input matrix
%     f0 : f(X_trim, u_trim) (should be ~0 at perfect trim)

% Dimensions
nx = length(X_trim);  % should be 6
nu = length(u_trim);  % should be 2
A = zeros(nx, nx);
B = zeros(nx, nu);

% Evaluate dynamics at trim
elevator_trim = u_trim(1);
throttle_trim = u_trim(2);
f0 = flightSimulation(0, X_trim, params, elevator_trim, throttle_trim);

% Finite difference step sizes
% State perturbations
dx = [ ...
    1.0;        % x [m]        (position, not very critical)
    1.0;        % h [m]
    0.1;        % Vx [m/s]
    0.1;        % Vz [m/s]
    0.001;      % theta [rad]
    0.001       % q [rad/s]
];

% Input perturbations
du = [ ...
    0.001;      % elevator [rad]
    0.001       % throttle [-]
];

% Compute A matrix via central differences in states
for i = 1:nx
    dX = zeros(nx,1);
    dX(i) = dx(i);
    X_plus  = X_trim + dX;
    X_minus = X_trim - dX;
    f_plus  = flightSimulation(0, X_plus,  params, elevator_trim, throttle_trim);
    f_minus = flightSimulation(0, X_minus, params, elevator_trim, throttle_trim);
    A(:,i) = (f_plus - f_minus) / (2*dx(i));
end

% Compute B matrix via central differences in inputs
for j = 1:nu
    dU = zeros(nu,1);
    dU(j) = du(j);
    u_plus  = u_trim + dU;
    u_minus = u_trim - dU;
    elev_plus   = u_plus(1);
    thrott_plus = u_plus(2);
    elev_minus   = u_minus(1);
    thrott_minus = u_minus(2);
    f_plus  = flightSimulation(0, X_trim, params, elev_plus,  thrott_plus);
    f_minus = flightSimulation(0, X_trim, params, elev_minus, thrott_minus);
    B(:,j) = (f_plus - f_minus) / (2*du(j));
end

end