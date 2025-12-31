% ============================================
% elevatorController.m (elevator PID controller)
% ============================================
function elevator_cmd = elevatorController(theta_cmd, theta, q, dt, ...
                                           elevator_trim, Kp, Ki, Kd)
    persistent integral prev_error last_theta_cmd
    
    if isempty(integral)
        integral = 0;
        prev_error = 0;
        last_theta_cmd = theta_cmd;
    end
    
    % Pitch error
    error = theta_cmd - theta;
    
    % Detect large command step (mode switch)
    cmd_change = abs(theta_cmd - last_theta_cmd);
    if cmd_change > 2*pi/180  % More than 2 degrees change
        % Bumpless transfer: adjust integral to prevent jump
        % Calculate what control was just before
        old_control = Kp*prev_error + Ki*integral - Kd*q;
        % Calculate what control would be with new command
        new_p_term = Kp*error;
        new_d_term = -Kd*q;
        % Adjust integral so total control stays same
        integral = (old_control - new_p_term - new_d_term) / Ki;
    end
    
    % Integral with anti-windup
    integral = integral + error * dt;
    
    % Adaptive windup limits based on error magnitude
    if abs(error) > 5*pi/180  % Large error (>5 deg)
        int_limit = 30;
    else
        int_limit = 10;
    end
    integral = max(min(integral, int_limit), -int_limit);
    
    % Derivative (uses pitch rate directly)
    derivative = -q;
    
    % PID output
    control = -(Kp*error + Ki*integral + Kd*derivative);
    
    % Add trim offset
    elevator_cmd = elevator_trim + control;
    
    % Saturation with back-calculation anti-windup
    saturated_cmd = max(min(elevator_cmd, 15*pi/180), -15*pi/180);
    if saturated_cmd ~= elevator_cmd
        % We hit a limit - back off the integrator
        integral = integral - 0.5 * (elevator_cmd - saturated_cmd) * dt / Ki;
    end
    elevator_cmd = saturated_cmd;
    
    % Save for next iteration
    prev_error = error;
    last_theta_cmd = theta_cmd;
end
