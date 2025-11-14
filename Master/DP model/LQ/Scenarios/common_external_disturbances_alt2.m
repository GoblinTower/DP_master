% File containing the external disturbances common to all simulation runs

% Add seed (to ensure same disturbances at every run)
rng(42,"twister");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% External disturbances %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Current
current_variance = [1e3; 1e3; 1e3];
current_variance = [0e1; 0e1; 0e1];
% current_variance = [0; 0; 0];
current_start_values = [1e5; 2e5; 2e5];

current_force = zeros(3,N);
% Gaussian random walk
if (use_current_force)
    current_force(:,1) = current_start_values;
    for k=2:N
        current_force(:,k) = current_force(:,k-1) + normrnd(0, current_variance, 3, 1);
    end
end

% Wave
wave_variance = [1e2; 1e2; 1e2];
% wave_variance = [0; 0; 0];
wave_start_values = [1e4; 3e4; 1e4];

wave_force = zeros(3,N);
% Gaussian random walk
if (use_wave_force)
    wave_force(:,1) = wave_start_values;
    for k=2:N
        wave_force(:,k) = wave_force(:,k-1) + normrnd(0, wave_variance, 3, 1);
    end
end

% Wind parameters
rho = 1.247;        % [kg/m^3] - This is air density at 10 degrees
Af = 180.0;         % Frontal projected area 
Al = 311.0;         % Lateral projected area
L = 76.2;           % Length overall (total length from bow to stern)
Cx = 0.7;           % Wind coefficient with respect to surge
Cy = 0.825;         % Wind coefficient with respect to sway
Cn = 0.125;         % Wind coefficient with respect to yaw

% Estimation of beta (angle of attack) and wind velocity
wind_variance = [0.1; 0.2]*dt;
wind_start_values = [deg2rad(35); 10];

wind = zeros(2,N);
% Gaussian random walk
if (use_wind_force)
    wind(:,1) = [wind_start_values(1); wind_start_values(2)];
    for k=2:N
        wind(:,k) = wind(:,k-1) + normrnd(0, wind_variance, 2, 1);
    end
end

wind_beta = smooth(wind(1,:));
wind_abs = smooth(wind(2,:));
