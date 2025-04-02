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
Q = diag([1e8, 1e8, 1e10]);          % Error weighting matrix
P = diag([1e-6, 1e-6, 1e-6]);        % Input weighting matrix

options = optimoptions('fmincon', 'display', 'off');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

% Setpoints [North, East, Yaw]
setpoint = zeros(3, N + horizon_length - 1);
n_setpoint = size(setpoint, 2);
for k=1:n_setpoint
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 300)
        setpoint(:,k) = [10; 5; deg2rad(90)];
    else
        setpoint(:,k) = [-5; -10; deg2rad(270)];
    end
end

% Kalman filter (extended)
run_kalman_filter = true;

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

% Measurement noise
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];
