
% ============================================
% verticalSpeedController.m (vertical speed PID controller)
% ============================================
function theta_cmd = verticalSpeedController(target_Vz, Vz, dt, theta_trim, ...
                                             Kp, Ki, Kd)
    persistent int_err prev_error
    
    if isempty(int_err)
        int_err = 0;
        prev_error = 0;
    end
    
    % Vertical speed error
    error = target_Vz - Vz;
    
    % Conditional integration - only integrate when we're not saturated
    % and error is reasonable
    if abs(error) < 3  % Within 3 m/s (≈600 fpm) of target
        int_err = int_err + error * dt;
        % Tighter anti-windup when close to target
        int_err = max(min(int_err, 3), -3);
    elseif abs(error) < 8  % Moderate error
        int_err = int_err + error * dt;
        int_err = max(min(int_err, 8), -8);
    else
        % Large error aggressive but doesnt let it build up too much
        int_err = int_err + 0.5 * error * dt;
        int_err = max(min(int_err, 10), -10);
    end
    
    % Derivative
    derivative = (error - prev_error) / dt;
    
    % PID output
    delta_theta = Kp*error + Ki*int_err + Kd*derivative;
    
    % Aggressive limits for descent (need authority)
    delta_theta = max(min(delta_theta, 5*pi/180), -15*pi/180);
    
    theta_cmd = theta_trim + delta_theta;
    
    % Rate limiting (max 3 deg/sec for more aggressive descent)
    persistent last_theta_cmd
    if isempty(last_theta_cmd)
        last_theta_cmd = theta_trim;
    end
    
    max_rate = 3 * pi/180 * dt;
    delta_cmd = theta_cmd - last_theta_cmd;
    if abs(delta_cmd) > max_rate
        theta_cmd = last_theta_cmd + sign(delta_cmd) * max_rate;
    end
    
    % Back-calculation anti-windup
    saturated_theta = max(min(theta_cmd, theta_trim + 5*pi/180), ...
                          theta_trim - 15*pi/180);
    if saturated_theta ~= theta_cmd
        % Unwind integral when saturated
        int_err = int_err - 0.3 * (theta_cmd - saturated_theta) / (Ki * dt);
    end
    theta_cmd = saturated_theta;
    
    last_theta_cmd = theta_cmd;
    prev_error = error;
end