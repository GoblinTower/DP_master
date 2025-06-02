function force = wind_force_calc(v_wind, beta, psi, u, v, rho, Af, Al, L, Cx, Cy, Cn)
    % Function for calculating wind forces on ship
    % See "Handbook of Marine Craft Hydrodynamics and Motion Control" by
    % Thor I. Fossen for a more detailed treatment of the theory applied
    % here.

    % v_w  : wind velocity (absolute value)
    % beta : angle of attack of wind
    % psi  : ship heading relative to North (measured clockwise)
    % u    : ship velocity in surge
    % v    : ship velocity in sway
    % rho  : density of air
    % Af   : frontal projected area 
    % Al   : lateral projected area
    % L    : length overall (total length from bow to stern)
    % Cx   : wind coefficient with respect to surge
    % Cy   : wind coefficient with respect to sway
    % Cn   : wind coefficient with respect to yaw
    
    force = zeros(3,1);

    % Must calculate wind speed relative to BODY coordinates
    u_w = v_wind*cos(beta - psi);
    v_w = v_wind*sin(beta - psi);

    % Must calculate relative speed of wind
    u_rw = u - u_w;
    v_rw = v - v_w;

    W_rw = sqrt(u_rw^2 + v_rw^2);

    % Calculate relative angle of attack with respect to bow
    gamma_rw = -atan2(v_rw, u_rw);

    % Dynamic pressure of the apparent wind
    q = 0.5*rho*W_rw^2;

    % Wind force in surge
    force(1) = -q*Cx*cos(gamma_rw)*Af;

    % Wind force in sway
    force(2) = q*Cy*sin(gamma_rw)*Al;

    % Momentum in yaw
    force(3) = q*Cn*sin(2*gamma_rw)*Al*L;

end