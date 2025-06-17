% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The control signal is calculated
% using a non-linear MPC.

dt = 1.0;           % Timestep used in integration
T = 300;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler

% integration_method = IntegrationMethod.Forward_Euler;
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/simple_tracking_no_dist";           % Name of folder to store output files
file_prefix = "nmpc_no_dist_simple_tracking_";        % Prefix of file names

% MPC control parameters
horizon_length = 10;                 % Prediction horizon length
Q = diag([1e14, 1e14, 1e16]);        % Error weighting matrix
Q = diag([1e12, 1e12, 1e16]);        % Error weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

options = optimoptions('fmincon', 'display', 'off');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

% Setpoints [North, East, Yaw]
% setpoint = zeros(3, N + horizon_length - 1);
% n_setpoint = size(setpoint, 2);
% for k=1:n_setpoint
%     time = k*dt;
%     if (time < 20)
%         setpoint(:,k) = [0; 0; 0];
%     elseif (time < 100)
%         setpoint(:,k) = [10; 5; deg2rad(30)];
%     else
%         setpoint(:,k) = [-5; -5; deg2rad(45)];
%     end
% end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

W = diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e12, 1e12, 1e12]);     % Process noise     
V = 10*eye(3);                                                  % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];       % Initial state estimate
n_kal_dim = size(x0_est,1);                 % Number of states in Kalman filter
G_lin = eye(n_kal_dim);                     % Process noise matrix

p_aposteriori = 1.0*eye(9);                 % Aposteriori covariance estimate
x_aposteriori = x0_est;                     % Aposteriori state estimate

animate_kalman_estimate = true;             % Animate kalman estimate
animation_delay = 0;                        % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
% x = [x, y, psi, u, v, r]
% y = [x, y, psi]

x0 = [0; 0; 0; 0; 0; 0];                    % Initial values of states (real)
n_dim = size(x0,1);                         % Number of states in Kalman filter

y0_meas = x0(1:3);                          % Initial values of measurements

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurement noise %%%
%%%%%%%%%%%%%%%%%%%%%%%%%
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter. The vector represents measurement 
% error in north position, east position and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Waypoint following %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
distance_to_update_setpoint = 5;
angle_to_update_setpoint = deg2rad(2);

waypoints = [20, 80, 100, 160, 200, 240, 300, 380, 400, 460; 
             10, 60, 100, 60, 120, 180, 200, 300, 240, 260];

waypoints = calculate_angles_waypoints(waypoints, x0(1:2));

last_waypoint_index = size(waypoints, 2);

show_path_plot = false;
if (show_path_plot)
    figure(1);
    hold on;
    plot(waypoints(2,:), waypoints(1,:), 'bo-');
    title("waypoints");
    xlabel("East");
    ylabel("North");
    legend({'Waypoints'}, 'Location', 'Best');
    grid();
    hold off;
end

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = false;
use_wave_force = false;
use_wind_force = false;

% Current
current_variance = [1e3; 1e3; 0];
current_variance = [0; 0; 0];
current_start_values = [1e5; 2e5; 0];

current_force = zeros(3,N);
% Gaussian random walk
if (use_current_force)
    current_force(:,1) = current_start_values;
    for j=2:N
        current_force(:,j) = current_force(:,j-1) + normrnd(0, current_variance, 3, 1);
    end
end

% Wave
wave_variance = [1e2; 1e2; 1e2];
wave_variance = [0; 0; 0];
wave_start_values = [1e4; 3e4; 5e2];

wave_force = zeros(3,N);
% Gaussian random walk
if (use_wave_force)
    wave_force(:,1) = wave_start_values;
    for j=2:N
        wave_force(:,j) = wave_force(:,j-1) + normrnd(0, wave_variance, 3, 1);
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
    for j=2:N
        wind(:,j) = wind(:,j-1) + normrnd(0, wind_variance, 2, 1);
    end
end

wind_beta = smooth(wind(1,:));
wind_abs = smooth(wind(2,:));

