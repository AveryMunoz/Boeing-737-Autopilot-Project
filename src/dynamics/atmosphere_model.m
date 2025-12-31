% Simple exponential air density model to help simulate realistic flight

function [airDensity, T] = atmosphere_model(h, atm)

    airDensity = atm.rho * exp(-h / atm.H);

        % Temperature model (ISA lapse rate) =
    T_sl = atm.T_sl;      
    lapse = -0.0065;      

    if h <= 11000
        T = T_sl + lapse * h;
    else
        T = 216.65;       % Constant above 11 km
    end
    
end