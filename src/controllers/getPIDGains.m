% ============================================
% IMPROVED getPIDGains.m
% ============================================
function [Kp, Ki, Kd] = getPIDGains(phase_name, controller_type)
    switch phase_name
        case 'climb_1'
            if strcmp(controller_type, 'elevator')
                Kp = 0.8;
                Ki = 0.05;
                Kd = 0.2;
            else  % throttle
                Kp = 0.25;
                Ki = 0.02;
                Kd = 0.05;
            end
            
        case 'climb_2'
            if strcmp(controller_type, 'elevator')
                Kp = 0.75;
                Ki = 0.045;
                Kd = 0.18;
            else
                Kp = 0.22;
                Ki = 0.018;
                Kd = 0.045;
            end
            
        case 'climb_3'
            if strcmp(controller_type, 'elevator')
                Kp = 0.7;
                Ki = 0.04;
                Kd = 0.15;
            else
                Kp = 0.20;
                Ki = 0.015;
                Kd = 0.04;
            end
            
        case 'cruise'
            if strcmp(controller_type, 'elevator')
                Kp = 0.75;    
                Ki = 0.5;   
                Kd = 0.5;   
            else  % throttle
                Kp = 0.15;   
                Ki = 0.01;   
                Kd = 0.03;   
            end
            
        case 'descent'
            if strcmp(controller_type, 'elevator')
                Kp = 0.7;
                Ki = 0.04;
                Kd = 0.15;
            else
                Kp = 0.18;   
                Ki = 0.015;
                Kd = 0.04;
            end
    end
end