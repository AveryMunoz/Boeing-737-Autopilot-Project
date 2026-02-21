addpath(genpath(pwd));

clearvars;
clear functions;
close all;
clc;
clear elevatorController throttleController altitudeController verticalSpeedController

run('ParametersClimb.m');
load('all_flight_trims.mat', 'trim_data');

fprintf('\n FULL FLIGHT SIMULATION \n');
fprintf('Simulating complete flight profile:\n');
fprintf('  - Climb Phase 1: Ground to 10,000 ft\n');
fprintf('  - Climb Phase 2: 10,000 to 28,000 ft\n');
fprintf('  - Climb Phase 3: 28,000 to 35,000 ft\n');
fprintf('  - Cruise: 35,000 ft for 2 hours\n');
fprintf('  - Descent: 35,000 ft to 3,000 ft\n\n');

% Simulation settings
dt = 0.1;  % Time step
cruise_duration = 7200;  % 2 hours in seconds
descent_target_alt = 3000 * 0.3048;  % 3000 ft in meters

% Initialize from climb_1 trim
X = trim_data.climb_1.X_trim;
X(2) = 1000 * 0.3048;  % Start at 100 ft

% Pre-allocate large array (estimate ~2.5 hours total)
max_steps = 150000;
log = zeros(max_steps, 19);

% Flight phase tracking
current_phase = 'climb_1';
cruise_entry_time = NaN;
cruise_entry_Vz = NaN;
descent_initiated = false;
descent_entry_time = NaN;

% Outer-loop PID gains for cruise and descent
Kp_h = 0.002;
Ki_h = 0.01;
Kd_h = 0.002;

Kp_vz = 0.001;
Ki_vz = 0.001;
Kd_vz = 0.005;

CRUISE_ALT_FT     = 35000;
CRUISE_ENTER_BAND = 200;     % ft - enter cruise slightly early
CRUISE_EXIT_BAND  = 200;    

cruise_latched = false;

% Cruise transition settings
transition_duration = 60;  
cruise_speed_ramp = 159.5; 

% Clear persistent variables in controllers
clear elevatorController throttleController altitudeController verticalSpeedController

k = 1;
while k <= max_steps
    % Extract current state
    h = X(2);
    Vx = X(3);
    Vz = X(4);
    theta = X(5);
    q = X(6);
    
    t = (k-1) * dt;
    h_ft = h / 0.3048;
    
    previous_phase = current_phase;

    % Phase logic with descent triggering
    if ~descent_initiated
        % Check if cruise duration exceeded
        if cruise_latched && ~isnan(cruise_entry_time)
            time_in_cruise = t - cruise_entry_time;
            if time_in_cruise >= cruise_duration
                descent_initiated = true;
                descent_entry_time = t;
                fprintf('\n✓ Initiating descent after %.1f hours in cruise\n', time_in_cruise/3600);
            end
        end
        
        % Normal climb/cruise phase logic
        if cruise_latched
            current_phase = 'cruise';
        else
            if h_ft < 10000
                current_phase = 'climb_1';
            elseif h_ft < 28000
                current_phase = 'climb_2';
            elseif h_ft < (CRUISE_ALT_FT - CRUISE_ENTER_BAND)
                current_phase = 'climb_3';
            else
                current_phase = 'cruise';
                cruise_latched = true;
                cruise_entry_time = t;
                cruise_entry_Vz = Vz;  % Capture entry vertical speed
            end
        end
    else
        % In descent phase
        current_phase = 'descent';
        
        % Check if descent target reached
        if h_ft <= (descent_target_alt / 0.3048)
            fprintf('\n✓ Descent target altitude reached: %.0f ft\n', h_ft);
            fprintf('Terminating simulation.\n');
            break;
        end
    end
    
    % Notify phase transitions
    if ~strcmp(current_phase, previous_phase)
        fprintf('→ Phase transition: %s → %s at t=%.1f s (alt=%.0f ft)\n', ...
                previous_phase, current_phase, t, h_ft);
        
        if strcmp(current_phase, 'cruise')
            % Only clear altitude controller to preserve elevator integral term
            clear altitudeController
            fprintf('  Starting %d-second cruise transition...\n', transition_duration);
        end
        
        if strcmp(current_phase, 'descent')
            % Clear controllers for descent
            clear altitudeController verticalSpeedController
            fprintf('  Starting descent to %.0f ft...\n', descent_target_alt / 0.3048);
        end
    end
    
    % Phase Parameters
    switch current_phase
        case 'climb_1'
            target_Vx = 250 * 0.5144;
            target_Vz = 2000/60 * 0.3048;
            theta_trim = trim_data.climb_1.X_trim(5);
            u_trim = trim_data.climb_1.u_trim;
            
        case 'climb_2'
            target_Vx = 293 * 0.5144;
            target_Vz = 1800/60 * 0.3048;
            theta_trim = trim_data.climb_2.X_trim(5);
            u_trim = trim_data.climb_2.u_trim;
            
        case 'climb_3'
            target_Vx = 310 * 0.5144;
            target_Vz = 1000/60 * 0.3048;
            theta_trim = trim_data.climb_3.X_trim(5);
            u_trim = trim_data.climb_3.u_trim;
            
        case 'cruise'
            target_Vz = 0;
            h_target = trim_data.cruise.X_trim(2);
            theta_trim = trim_data.cruise.X_trim(5);
            u_trim = trim_data.cruise.u_trim;
            
            % Gradual transition over transition_duration seconds
            time_in_cruise = t - cruise_entry_time;
            
            if time_in_cruise < transition_duration
                % Gradually reduce vertical speed from climb to zero
                progress = time_in_cruise / transition_duration;
                target_Vz_transition = cruise_entry_Vz * (1 - progress);
                
                % Use vertical speed controller during transition
                theta_cmd = verticalSpeedController(target_Vz_transition, Vz, dt, ...
                                                    theta_trim, Kp_vz, Ki_vz, Kd_vz);
                
                % Keep climb speed during transition
                target_Vx = 310 * 0.5144;
            else
                % Transition complete - switch to altitude hold
                theta_cmd = altitudeController(h_target, h, Vz, dt, theta_trim, ...
                                               Kp_h, Ki_h, Kd_h);
                
                % Now ramp up to cruise speed
                cruise_final_speed = 248.976;
                if cruise_speed_ramp < cruise_final_speed
                    cruise_speed_ramp = cruise_speed_ramp + 0.1*dt;
                end
                target_Vx = cruise_speed_ramp;
            end
            
        case 'descent'
        u_trim = trim_data.descent.u_trim;
        theta_trim = trim_data.descent.X_trim(5);
        
        % Altitude-based descent profile
        if h_ft > 10000
            target_Vz = -1500/60 * 0.3048;
            target_Vx = 250 * 0.5144;  % Normal speed
        elseif h_ft > 5000
            target_Vz = -1000/60 * 0.3048;
            target_Vx = 200 * 0.5144;  % Slower to reduce lift
        else
            target_Vz = -500/60 * 0.3048;
            target_Vx = 180 * 0.5144;  % Even slower near ground
        end
    end
    % Contorller Gains 
    [Kp_elev, Ki_elev, Kd_elev] = getPIDGains(current_phase, 'elevator');
    [Kp_throt, Ki_throt, Kd_throt] = getPIDGains(current_phase, 'throttle');
    
    % Controller Logic
    if strcmp(current_phase, 'cruise')
        % theta_cmd already set in the cruise case above
        elevator_cmd = elevatorController(theta_cmd, theta, q, dt, ...
                                         u_trim(1), Kp_elev, Ki_elev, Kd_elev);
        
    elseif strcmp(current_phase, 'descent')
        % Vertical speed hold mode
        theta_cmd = verticalSpeedController(target_Vz, Vz, dt, theta_trim, ...
                                            Kp_vz, Ki_vz, Kd_vz);
        elevator_cmd = elevatorController(theta_cmd, theta, q, dt, ...
                                         u_trim(1), Kp_elev, Ki_elev, Kd_elev);
    else
        % Climb phases: pitch hold
        elevator_cmd = elevatorController(theta_trim, theta, q, dt, ...
                                         u_trim(1), Kp_elev, Ki_elev, Kd_elev);
    end
    
    % Throttle control
    throttle_cmd = throttleController(target_Vx, Vx, dt, ...
                                     u_trim(2), Kp_throt, Ki_throt, Kd_throt);
    
    % Simulate one step
    X = rk4_step(@flightSimulation, X, dt, params, elevator_cmd, throttle_cmd);
    
    if h < 0 || h > 60000
        fprintf('WARNING: Altitude out of bounds (%.0f ft), stopping simulation\n', h_ft);
        break;
    end
    
    % Logging values
    forces = calculateForces(X, params, elevator_cmd, throttle_cmd);
    V_total = sqrt(X(3)^2 + X(4)^2);
    
    % Store phase as number for logging
    phase_num = 0;
    switch current_phase
        case 'climb_1', phase_num = 1;
        case 'climb_2', phase_num = 2;
        case 'climb_3', phase_num = 3;
        case 'cruise', phase_num = 4;
        case 'descent', phase_num = 5;
    end
    
    log(k,:) = [t, h, Vx, Vz, theta, q, ...
                elevator_cmd, throttle_cmd, target_Vx, target_Vz, ...
                forces.Lift, forces.Drag, forces.Thrust, ...
                forces.alpha, forces.CL, forces.CD, ...
                V_total, forces.M, phase_num];   
    k = k + 1;
    
    % Progress update every 60 seconds
    if mod(k, 600) == 0
        fprintf('  t=%.1f min: Phase=%s, Alt=%.0f ft, Vx=%.1f m/s, Vz=%.1f fpm\n', ...
                t/60, current_phase, h_ft, Vx, Vz*60/0.3048);
    end
end

% Trim log to actual length
log = log(1:k-1,:);

fprintf('\n SIMULATION COMPLETE \n');
fprintf('Total simulation time: %.1f minutes (%.2f hours)\n', log(end,1)/60, log(end,1)/3600);

% Data Extraction
t = log(:,1);
h_log = log(:,2);
Vx_log = log(:,3);
Vz_log = log(:,4);
theta_log = log(:,5);
q_log = log(:,6);
elev_log = log(:,7);
throt_log = log(:,8);
target_Vx_log = log(:,9);
target_Vz_log = log(:,10);
Lift_log = log(:,11);
Drag_log = log(:,12);
Thrust_log = log(:,13);
alpha_log = log(:,14);
CL_log = log(:,15);
CD_log = log(:,16);
V_total_log = log(:,17);
M_log = log(:,18);
phase_log = log(:,19);

% Derived quantities
gamma_log = atan2(Vz_log, Vx_log);
Weight = params.airplane.mass * params.airplane.gravity;
L_over_W = Lift_log / Weight;
T_over_D = Thrust_log ./ Drag_log;

% PLOTS
figure('Position', [50 50 1400 1000]);

% sample every 1 minute
plot_sample = 1:600:length(t);  % Every 60 seconds (600 * 0.1s)

% Extract sampled data
t_plot = t(plot_sample);
h_plot = h_log(plot_sample);
Vx_plot = Vx_log(plot_sample);
Vz_plot = Vz_log(plot_sample);
target_Vx_plot = target_Vx_log(plot_sample);
target_Vz_plot = target_Vz_log(plot_sample);
theta_plot = theta_log(plot_sample);
phase_plot = phase_log(plot_sample);
elev_plot = elev_log(plot_sample);
throt_plot = throt_log(plot_sample);
L_over_W_plot = L_over_W(plot_sample);
Thrust_plot = Thrust_log(plot_sample);
Drag_plot = Drag_log(plot_sample);
alpha_plot = alpha_log(plot_sample);

% Plot 1: Altitude Profile
subplot(3,3,1);
plot(t_plot/60, h_plot*3.281, 'b', 'LineWidth', 2); grid on;
xlabel('Time [min]'); ylabel('Altitude [ft]');
title('Full Flight Altitude Profile');
yline(10000, 'r--', 'LineWidth', 1);
yline(28000, 'r--', 'LineWidth', 1);
yline(35000, 'r--', 'LineWidth', 1);
yline(3000, 'g--', 'LineWidth', 1);

% Plot 2: Horizontal Speed
subplot(3,3,2);
plot(t_plot/60, Vx_plot, 'b', 'LineWidth', 2); hold on;
plot(t_plot/60, target_Vx_plot, 'r--', 'LineWidth', 1.5); grid on;
xlabel('Time [min]'); ylabel('Vx [m/s]');
title('Horizontal Speed Tracking');
legend('Actual', 'Target', 'Location', 'best');

% Plot 3: Vertical Speed 
subplot(3,3,3);
plot(t_plot/60, Vz_plot*60/0.3048, 'b', 'LineWidth', 2); hold on;
plot(t_plot/60, target_Vz_plot*60/0.3048, 'r--', 'LineWidth', 1.5); grid on;
xlabel('Time [min]'); ylabel('Vz [fpm]');
title('Vertical Speed Tracking');
legend('Actual', 'Target', 'Location', 'best');
ylim([-10000, 10000]);  

% Plot 4: Flight Phase
subplot(3,3,4);
plot(t_plot/60, phase_plot, 'k', 'LineWidth', 2); grid on;
xlabel('Time [min]'); ylabel('Phase');
title('Flight Phase');
yticks(1:5);
yticklabels({'Climb1', 'Climb2', 'Climb3', 'Cruise', 'Descent'});

% Plot 5: Pitch Angle 
subplot(3,3,5);
plot(t_plot/60, theta_plot*180/pi, 'b', 'LineWidth', 2); grid on;
xlabel('Time [min]'); ylabel('Theta [deg]');
title('Pitch Angle');
ylim([-45, 45]);  

% Plot 6: Control Surfaces
subplot(3,3,6);
plot(t_plot/60, elev_plot*180/pi, 'b', 'LineWidth', 2);
ylabel('Elevator [deg]');
xlabel('Time [min]'); grid on;
title('Control Inputs');

% Plot 7: Lift vs Weight 
subplot(3,3,7);
plot(t_plot/60, L_over_W_plot, 'b', 'LineWidth', 2); hold on;
yline(1.0, 'r--', 'LineWidth', 1.5); grid on;
xlabel('Time [min]'); ylabel('L/W');
title('Lift-to-Weight Ratio');
legend('L/W', 'Ideal', 'Location', 'best');
ylim([-2.5, 5]);  

% Plot 8: Thrust vs Drag 
subplot(3,3,8);
plot(t_plot/60, Thrust_plot/1000, 'g', 'LineWidth', 2); hold on;
plot(t_plot/60, Drag_plot/1000, 'r', 'LineWidth', 2); grid on;
xlabel('Time [min]'); ylabel('Force [kN]');
title('Thrust and Drag');
legend('Thrust', 'Drag', 'Location', 'best');
ylim([-250, 500]);  

% Plot 9: Angle of Attack 
subplot(3,3,9);
plot(t_plot/60, alpha_plot*180/pi, 'b', 'LineWidth', 2); hold on;
yline(params.airplane.alpha_stall*180/pi, 'r--', 'LineWidth', 1.5); grid on;
xlabel('Time [min]'); ylabel('Alpha [deg]');
title('Angle of Attack');
legend('Alpha', 'Stall', 'Location', 'best');
ylim([-50, 50]);  

% PHASE STATISTICS 
fprintf('\n PHASE-BY-PHASE STATISTICS \n');

phase_names_full = {'climb_1', 'climb_2', 'climb_3', 'cruise', 'descent'};
for p = 1:5
    phase_mask = (phase_log == p);
    if any(phase_mask)
        fprintf('\n--- %s ---\n', upper(phase_names_full{p}));
        fprintf('  Duration: %.1f min\n', sum(phase_mask)*dt/60);
        fprintf('  Altitude change: %.0f ft\n', ...
                (max(h_log(phase_mask)) - min(h_log(phase_mask)))*3.281);
        fprintf('  Mean Vx: %.2f m/s (target: %.2f)\n', ...
                mean(Vx_log(phase_mask)), mean(target_Vx_log(phase_mask)));
        fprintf('  Mean Vz: %.2f m/s (%.0f fpm)\n', ...
                mean(Vz_log(phase_mask)), mean(Vz_log(phase_mask))*60/0.3048);
        fprintf('  Mean L/W: %.3f\n', mean(L_over_W(phase_mask)));
        fprintf('  Mean T/D: %.3f\n', mean(T_over_D(phase_mask)));
        fprintf('  Mean Alpha: %.2f deg\n', mean(alpha_log(phase_mask))*180/pi);
    end
end

% Save high-quality figure
print('flight_simulation_results', '-dpng', '-r300');