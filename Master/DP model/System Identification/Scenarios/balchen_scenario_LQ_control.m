% Configuration script: LQ optimal control of supply model

% Add seed (meaning og life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 2000;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% LQ control parameters
Q = diag([1e9, 1e9, 1e15]);          % State weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

% Setpoints [North, East, Yaw]
setpoint = zeros(3,N+1);
for k=1:N
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 1000)
        setpoint(:,k) = [10; 5; deg2rad(30)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(45)];
    end
end

% Kalman filter
run_kalman_filter = true;

W = 1.0*eye(9);                 % Process noise
V = 10.0*eye(3);                % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];    % Initial state estimate

p_aposteriori = 1.0*eye(9);     % Aposteriori covariance estimate
x_aposteriori = x0_est;         % Aposteriori state estimate

animate_kalman_estimate = true; % Animate kalman estimate
animation_delay = 0.01;         % Animation speed (in seconds)

% Initial values
% x = [x, y, psi, x_su, x_sw u, v, r]
x0 = [0; 0; 0; 0; 0; 0; 0; 0];          % Initial values of states (real)
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
