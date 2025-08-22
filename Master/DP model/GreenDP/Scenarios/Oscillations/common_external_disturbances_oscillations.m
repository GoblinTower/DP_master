% File containing the external disturbances common to all simulation runs

% Add seed (to ensure same disturbances at every run)
rng(42,"twister");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% External disturbances %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Current
current_variance = [5e3; 5e3; 0];
current_start_values = [0; 0; 0];
current_reduction_factor = 1.0;

current_force = zeros(3,N);
% Gaussian random walk
if (use_current_force)
    current_force(:,1) = current_start_values;
    for j=2:N
        current_force(:,j) = current_force(:,j-1) + normrnd(0, current_variance, 3, 1);
        current_force(:,j) = current_reduction_factor*current_force(:,j);
    end
end

% Waves
% First order wave drift
wave_variance = [1e3; 1e3; 1e3];
wave_start_values = [0; 0; 0];
wave_reduction_factor = 1.0;

% Second order wave oscillations
wave_period = 25;                    % Wave period in seconds
wave_direction_2_order = 0;          % Direction of wave oscillations (radians)
amplitude_force_2_order = 1e4;       % Wave force in Newton

wave_force = zeros(3,N);
wave_force_1_order = zeros(3,N);
wave_force_2_order = zeros(3,N);
% Gaussian random walk
if (use_wave_force)
    wave_force_1_order(:,1) = wave_start_values;
    for j=2:N
        
        % First order wave drift
        wave_force_1_order(:,j) = wave_force_1_order(:,j-1) + normrnd(0, wave_variance, 3, 1);
        
        % Second order wave drift
        wave_direction_2_order = atan2(wave_force_1_order(2,j), wave_force_1_order(1,j));       % Direction same as 1. order wave forces
        
        absolute_wave_force_2_order = cos((j*dt/25)*2*pi)*amplitude_force_2_order;              % Make force sinusoidal with constant period

        wave_force_2_order(:,j) = [cos(wave_direction_2_order)*absolute_wave_force_2_order; ...
            sin(wave_direction_2_order)*absolute_wave_force_2_order; 0];
    end
end

wave_force = wave_force_1_order + wave_force_2_order;
wave_force = wave_reduction_factor*wave_force;

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
    for j=2:N
        wind(:,j) = wind(:,j-1) + normrnd(0, wind_variance, 2, 1);
    end
end

wind_beta = smooth(wind(1,:));
wind_abs = smooth(wind(2,:));
