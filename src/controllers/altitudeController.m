% ============================================
% altitudeController.m (altitude PID Controller)
% ============================================
function theta_cmd = altitudeController(h_target, h, Vz, dt, theta_trim, ...
                                        Kp, Ki, Kd)
    persistent int_err prev_h first_call
    
    if isempty(int_err)
        int_err = 0;
        prev_h = h;
        first_call = true;
    end
    
    % Altitude error
    error = h_target - h;
    
    % Bumpless transfer on first call
    if first_call
        % Initialize so we start near trim pitch
        % Don't try to correct all error instantly
        int_err = 0;
        first_call = false;
    end
    
    % Deadband for very small errors (prevent hunting)
    if abs(error) < 3  % 3 meters ≈ 10 ft
        error = 0;
    end
    
    % Integral with conditional integration
    % Only integrate when error is moderate
    if abs(error) < 150  % 150m ≈ 500ft
        int_err = int_err + error * dt;
    end
    
    % Anti-windup
    int_err = max(min(int_err, 200), -200);
    
    % Derivative term uses Vz (rate of altitude change = vertical velocity)
    % Negative because positive Vz means altitude increasing
    derivative_term = -Vz;
    
    % Pitch command with rate limiting
    theta_cmd_raw = theta_trim + Kp*error + Ki*int_err + Kd*derivative_term;
    
    % Rate limit pitch commands to prevent over corrections (max 2 deg/sec change)
    persistent last_theta_cmd
    if isempty(last_theta_cmd)
        last_theta_cmd = theta_trim;
    end
    
    max_rate = 2 * pi/180 * dt;  % 2 deg/sec
    delta_theta = theta_cmd_raw - last_theta_cmd;
    if abs(delta_theta) > max_rate
        theta_cmd = last_theta_cmd + sign(delta_theta) * max_rate;
    else
        theta_cmd = theta_cmd_raw;
    end
    
    % Physical pitch limits relative to trim
    theta_cmd = max(min(theta_cmd, theta_trim + 5*pi/180), ...
                    theta_trim - 5*pi/180);
    
    last_theta_cmd = theta_cmd;
    prev_h = h;
end
