% Configuration script: NMPC control of supply model

% Add seed (meaning of life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 150;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler

% integration_method = IntegrationMethod.Forward_Euler;
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% MPC control parameters
horizon_length = 20;                 % Prediction horizon length
Q = diag([1e14, 1e14, 1e16]);        % Error weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

options = optimoptions('fmincon', 'display', 'off');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

% Kalman filter (extended)
V = 1.0*eye(3);                 % Process noise
W = 1.0*eye(9);                 % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];    % Initial state estimate

p_aposteriori = 1.0*eye(9);         % Aposteriori covariance estimate
x_aposteriori = x0_est;             % Aposteriori state estimate

animate_kalman_estimate = true; % Animate kalman estimate
animation_delay = 0;            % Animation speed (in seconds)

% Initial values
% x = [x, y, psi, u, v, r]
x0 = [0; 0; 0; 0; 0; 0];                % Initial values of states (real)
% y = [x, y, psi]
y0_meas = x0(1:3);                      % Initial values of measurements

% Path
distance_to_update_setpoint = 5;

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

% MPC thruster constraints
use_thruster_constraints = false;

max_delta_u = [1e7; 1e7; 2e9];
max_inputs = [5e7; 5e7; 1e10];

z_dim = 3 + 9 + 2*3;                    % r_dim + n_dim + 2*m_dim

% Measurement noise
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];
