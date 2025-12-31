function throttle_cmd = throttleController(target_Vx, current_Vx, dt, ...
                                           throttle_trim, Kp, Ki, Kd)
    
    persistent integral prev_error
    
    if isempty(integral)
        integral   = 0;
        prev_error = 0;
    end
    
    % Airspeed error
    error = target_Vx - current_Vx;
    
    % Integral with anti-windup
    integral = integral + error * dt;
    integral = max(min(integral, 10), -10);
    
    % Derivative (simple error rate, same style as elevatorController)
    derivative = (error - prev_error); % not deviding by dt as it is 1 and negligible
    
    prev_error = error;
    
    % PID output
    control = Kp*error + Ki*integral + Kd*derivative;
    
    % Add trim offset
    throttle_cmd = throttle_trim + control;
    
    % Saturation
    throttle_cmd = max(min(throttle_cmd, 1.0), 0.0);
end
