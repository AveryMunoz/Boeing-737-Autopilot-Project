clear; clc;
run('ParametersClimb.mlx');

% Initialize trim data storage
trim_data = struct();

% Define 3 climb phase targets

trim_targets(1).name = 'climb_1';
trim_targets(1).h = 5500 * 0.3048;
trim_targets(1).Vx = 250 * 0.5144;          % 250 KIAS
trim_targets(1).Vz = 2000/60 * 0.3048;      % 2500 fpm
trim_targets(1).gamma = atan2(trim_targets(1).Vz, trim_targets(1).Vx);
trim_targets(1).theta_guess = trim_targets(1).gamma + 3*pi/180;

trim_targets(2).name = 'climb_2';
trim_targets(2).h = 19000 * 0.3048;
trim_targets(2).Vx = 275 * 0.5144;          
trim_targets(2).Vz = 1800/60 * 0.3048;      % 1800 fpm = 9.14 m/s
trim_targets(2).gamma = atan2(trim_targets(2).Vz, trim_targets(2).Vx);
trim_targets(2).theta_guess = trim_targets(2).gamma + 3*pi/180;

trim_targets(3).name = 'climb_3';
trim_targets(3).h = 33500 * 0.3048;
trim_targets(3).Vx = 310 * 0.5144;          
trim_targets(3).Vz = 1000/60 * 0.3048;      % 1000 fpm = 5.08 m/s
trim_targets(3).gamma = atan2(trim_targets(3).Vz, trim_targets(3).Vx);
trim_targets(3).theta_guess = trim_targets(3).gamma + 2.5*pi/180;

trim_targets(4).name = 'cruise';
trim_targets(4).h = 35000 * 0.3048;          % 35,000 ft
trim_targets(4).Vx = 248.96;                 % Cruise speed (m/s)
trim_targets(4).Vz = 0;                      % Level flight
trim_targets(4).gamma = 0;                   % Level
trim_targets(4).theta_guess = 2*pi/180;      % Small pitch for cruise

trim_targets(5).name = 'descent';
trim_targets(5).h = 19000 * 0.3048;        % Mid-descent altitude
trim_targets(5).Vx = 250 * 0.5144;         % 250 KIAS
trim_targets(5).Vz = -1500/60 * 0.3048;    % -1500 fpm
trim_targets(5).gamma = atan2(trim_targets(5).Vz, trim_targets(5).Vx);
trim_targets(5).theta_guess = 0*pi/180;   %Start at 0° instead of -1°



% Loop through each phase

for i = 1:5
    fprintf('\n========== %s ==========\n', trim_targets(i).name);
    
    % Set initial state guess for this phase
    X_trim = [0; trim_targets(i).h; trim_targets(i).Vx; ...
              trim_targets(i).Vz; trim_targets(i).theta_guess; 0];
    
    % Phase-dependent initial control guess
    switch i
        case 1  % climb_1: low altitude, less power
            u_trim = [-2*pi/180; 0.60];
        case 2  % climb_2: medium altitude
            u_trim = [-1.5*pi/180; 0.65];
        case 3  % climb_3: high altitude, more power
            u_trim = [-1*pi/180; 0.75];
        case 4  % cruise: high altitude, moderate power
            u_trim = [0*pi/180; 0.55];  % Near-zero elevator, moderate throttle
        case 5  % descent: idle thrust, nose down
             u_trim = [-3*pi/180; 0.25]; 
    end
    
        % PHASE-SPECIFIC SEARCH RANGES
    if i == 5  % Descent at 19k ft
        theta_range = linspace(-2, 6, 25) * pi/180;
        elev_range = linspace(-4, 2, 25) * pi/180;
        throt_range = linspace(0.08, 0.20, 30);
    elseif i == 4  % Cruise
        theta_range = linspace(0, 6, 20) * pi/180;   % 0-6° instead of 0-4°
        elev_range = linspace(-3, 1, 20) * pi/180;   % Wider elevator range
        throt_range = linspace(0.48, 0.75, 20);     
    elseif i == 3  % Special for high altitude
        theta_range = linspace(2, 10, 20) * pi/180;   % Lower pitch (thinner air)
        elev_range = linspace(-6, 0, 20) * pi/180;   % More negative elevator allowed
        throt_range = linspace(0.70, 0.95, 20);      % Higher throttle range
    else
        theta_range = linspace(4, 10, 15) * pi/180;
        elev_range = linspace(-4, 3, 15) * pi/180;
        throt_range = linspace(0.45, 0.85, 15);
    end
    
    best_norm = inf;
    best_state = X_trim;
    best_u = u_trim;
    
    fprintf('Coarse search...\n');
    for ii = 1:length(theta_range)
        X_test = X_trim;
        X_test(5) = theta_range(ii);
        for jj = 1:length(elev_range)
            for kk = 1:length(throt_range)
                u_test = [elev_range(jj); throt_range(kk)];
                f_test = flightSimulation(0, X_test, params, u_test(1), u_test(2));
                norm_val = norm(f_test([3,4,6]));
                if norm_val < best_norm
                    best_norm = norm_val;
                    best_state = X_test;
                    best_u = u_test;
                end
            end
        end
    end
    
    fprintf('Coarse result - Norm: %.4f\n', best_norm);
    
    % FINE SEARCH
    theta_fine = linspace(best_state(5) - 1*pi/180, best_state(5) + 1*pi/180, 20);
    elev_fine = linspace(best_u(1) - 0.5*pi/180, best_u(1) + 0.5*pi/180, 20);
    throt_fine = linspace(best_u(2) - 0.02, best_u(2) + 0.02, 20);
    
    fprintf('Fine search...\n');
    for ii = 1:length(theta_fine)
        X_test = best_state;
        X_test(5) = theta_fine(ii);
        for jj = 1:length(elev_fine)
            for kk = 1:length(throt_fine)
                u_test = [elev_fine(jj); throt_fine(kk)];
                f_test = flightSimulation(0, X_test, params, u_test(1), u_test(2));
                norm_val = norm(f_test([3,4,6]));
                if norm_val < best_norm
                    best_norm = norm_val;
                    best_state = X_test;
                    best_u = u_test;
                end
            end
        end
    end

    % ULTRA-FINE SEARCH (only if needed)
    if best_norm > 0.01
        fprintf('Ultra-fine search...\n');
        theta_ultra = linspace(best_state(5) - 0.3*pi/180, best_state(5) + 0.3*pi/180, 15);
        elev_ultra = linspace(best_u(1) - 0.2*pi/180, best_u(1) + 0.2*pi/180, 15);
        throt_ultra = linspace(best_u(2) - 0.01, best_u(2) + 0.01, 15);
        
        for ii = 1:length(theta_ultra)
            X_test = best_state;
            X_test(5) = theta_ultra(ii);
            for jj = 1:length(elev_ultra)
                for kk = 1:length(throt_ultra)
                    u_test = [elev_ultra(jj); throt_ultra(kk)];
                    f_test = flightSimulation(0, X_test, params, u_test(1), u_test(2));
                    norm_val = norm(f_test([3,4,6]));
                    if norm_val < best_norm
                        best_norm = norm_val;
                        best_state = X_test;
                        best_u = u_test;
                    end
                end
            end
        end
    end

    X_trim = best_state;
    u_trim = best_u;
    
    % Calculate final values
    f0 = flightSimulation(0, X_trim, params, u_trim(1), u_trim(2));
    gamma_trim = atan2(X_trim(4), X_trim(3));
    alpha_trim = X_trim(5) - gamma_trim;

    fprintf('\nDiagnostic info:\n');
    fprintf('dVx/dt: %.4f m/s² (should be ~0)\n', f0(3));
    fprintf('dVz/dt: %.4f m/s² (should be ~0)\n', f0(4));
    fprintf('dq/dt: %.4f rad/s² (should be ~0)\n', f0(6));
    
    forces = calculateForces(X_trim, params, u_trim(1), u_trim(2));
    Weight = params.airplane.mass * params.airplane.gravity;  % Calculate it here
    fprintf('Thrust: %.0f N, Drag: %.0f N, T/D: %.2f\n', ...
        forces.Thrust, forces.Drag, forces.Thrust/forces.Drag);
    fprintf('Lift: %.0f N, Weight: %.0f N, L/W: %.2f\n', ...
        forces.Lift, Weight, forces.Lift/Weight);

    alpha_deg = alpha_trim * 180/pi;
    CL = params.airplane.CL_poly(1)*alpha_deg^2 + params.airplane.CL_poly(2)*alpha_deg + params.airplane.CL_poly(3);
    fprintf('CL at alpha=%.1f deg: %.3f\n', alpha_deg, CL);
    
    fprintf('\n=== FINAL TRIM ===\n');
    fprintf('Theta: %.4f deg\n', X_trim(5)*180/pi);
    fprintf('Gamma: %.4f deg\n', gamma_trim*180/pi);
    fprintf('Alpha: %.4f deg\n', alpha_trim*180/pi);
    fprintf('Elevator: %.4f deg\n', u_trim(1)*180/pi);
    fprintf('Throttle: %.6f\n', u_trim(2));
    fprintf('Norm: %.6f\n', best_norm);
    
    if best_norm > 0.01
        warning('Trim not fully converged.');
    end
    
    % Linearize
    [A, B, f0] = linearizeAircraft(X_trim, u_trim, params);
    
    % === SAVE RESULTS FOR THIS PHASE ===
    trim_data.(trim_targets(i).name).X_trim = X_trim;
    trim_data.(trim_targets(i).name).u_trim = u_trim;
    trim_data.(trim_targets(i).name).A = A;
    trim_data.(trim_targets(i).name).B = B;
    trim_data.(trim_targets(i).name).f0 = f0;
    trim_data.(trim_targets(i).name).norm = best_norm;
    trim_data.(trim_targets(i).name).alpha_trim = alpha_trim;
    trim_data.(trim_targets(i).name).gamma_trim = gamma_trim;
    
    fprintf('Stored: trim_%s\n', trim_targets(i).name);
end

% === SAVE ALL TRIM DATA TO FILE ===
save('all_flight_trims.mat', 'trim_data', 'trim_targets');

fprintf('\n ALL PHASES COMPLETE \n');
fprintf('All trim data saved to: all_flight_trims.mat\n');
fprintf('\nPhases saved:\n');
for i = 1:5
    fprintf('  - %s (norm: %.6f)\n', trim_targets(i).name, trim_data.(trim_targets(i).name).norm);
end

load('all_flight_trims.mat', 'trim_data');
fprintf('Cruise trim Vx: %.2f m/s\n', trim_data.cruise.X_trim(3));