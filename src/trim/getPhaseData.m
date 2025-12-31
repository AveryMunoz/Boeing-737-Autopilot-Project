function [phase_name, target_Vx, target_Vz, theta_cmd, h_target, trim_data] = getPhaseData(h, all_trims)
    % Determine current phase based on altitude
    % Returns phase-specific targets and trim data
    
    h_ft = h / 0.3048;  % Convert to feet
    
    if h_ft < 10000
        phase_name = 'climb_1';
        target_Vx = 250 * 0.5144;      % 250 KIAS
        target_Vz = 2000/60 * 0.3048;  % 2000 fpm
        
    elseif h_ft < 28000
        phase_name = 'climb_2';
        target_Vx = 293 * 0.5144;      % 293 KIAS
        target_Vz = 1800/60 * 0.3048;  % 1800 fpm

    elseif h_ft < 35000
        phase_name = 'climb_3';
        target_Vx = 325 * 0.5144;
        target_Vz = 1000/60 * 0.3048;
    elseif h_ft >= 35000
        phase_name = 'cruise';
        target_Vx = 248.976;           % Cruise speed
        target_Vz = 0;
    else  % h_ft >= 36000 (above cruise, start descending)
        phase_name = 'descent';
        target_Vx = 250 * 0.5144;
        target_Vz = -1500/60 * 0.3048;
    end
    
    trim_data = all_trims.(phase_name);
    theta_cmd = trim_data.X_trim(5);
    h_target = trim_data.X_trim(2);
end
