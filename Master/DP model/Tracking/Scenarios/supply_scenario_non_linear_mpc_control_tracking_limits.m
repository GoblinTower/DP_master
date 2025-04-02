% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The control signal is calculated
% using a non-linear MPC. The maximum thruster force is limited and
% together with the change in thruster force per time step.
% Time step has been increased as the thruster dynamics is slowed down.

dt = 1.0;           % Timestep used in integration

T = 100;             % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler

% integration_method = IntegrationMethod.Forward_Euler;
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% MPC control parameters
horizon_length = 25;                 % Prediction horizon length
Q = diag([1e8, 1e8, 1e10]);          % Error weighting matrix
P = diag([1e-6, 1e-6, 1e-6]);        % Input weighting matrix

% MPC thruster constraints
use_thruster_constraints = true;
% max_delta_u = [1e6; 1e6; 1e7];
% max_inputs = [1e8; 1e8; 5e9];
max_delta_u = [1e7; 1e7; 2e9];
max_inputs = [5e7; 5e7; 1e10];

options = optimoptions('fmincon', 'display', 'off');

% Path
distance_to_update_setpoint = 5;
show_path_plot = false;
waypoints = [20, 80, 100, 160, 200, 240, 300, 380, 400, 460; 
             10, 60, 100, 60, 120, 180, 200, 300, 240, 260];

last_waypoint_index = size(waypoints, 2);

ref_angle = zeros(last_waypoint_index, 1);
% for k=1:(last_waypoint_index-1)
%     ref_angle(k) = atan2(waypoints(2,k+1) - waypoints(2,k), waypoints(1,k+1) - waypoints(1,k));
% end
% ref_angle(end) = ref_angle(end-1);
for k=1:last_waypoint_index
    if (k == 1)
        ref_angle(k) = atan2(waypoints(2,k) - 0, waypoints(1,k) - 0);
    else
        ref_angle(k) = atan2(waypoints(2,k) - waypoints(2,k-1), waypoints(1,k) - waypoints(1,k-1));
    end
end

if (show_path_plot)
    figure(1);
    hold on;
    plot(waypoints(2,:), waypoints(1,:), 'bo-');
    title("waypoints");
    xlabel("East");
    ylabel("North");
    legend({'Original waypoints', 'New path'}, 'Location', 'Best');
    grid();
    hold off;
end

% Kalman filter
run_kalman_filter = true;

V = 1.0*eye(9);            % Process noise
W = 1.0*eye(3);            % Measurement noise
X_apriori = 1.0*eye(9);    % Apriori estimate covarianceS

% Supply model
% x = [x, y, psi, u, v, r]
x0 = [0; 0; 0; 0; 0; 0];                % Inital values of states

% Linearized model with integral gain
% x = [x, y, psi, u, v, r, b1, b2, b3]
x_lin0 = [0; 0; 0; 0; 0; 0; 0; 0; 0];   % Inital values of states

% Measurement noise
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.2; 0.2; 0.1];
