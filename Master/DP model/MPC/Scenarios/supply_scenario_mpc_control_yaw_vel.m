% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. MPC is used for maintaining setpoint.

% Add seed (meaning og life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 1500;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Select Mpc model
% MpcModel.ConstantStateMatrix
% MpcModel.ConstantYawRateMode
mpcModel = MpcModel.ConstantYawRateMode;

% MPC control parameters
horizon_length = 10;                 % Prediction horizon length
Q = diag([1e8, 1e8, 1e10]);          % Error weighting matrix
P = diag([1e-6, 1e-6, 1e-6]);        % Input weighting matrix

options = optimoptions('quadprog', 'display', 'off');

% Setpoints [North, East, Yaw]
setpoint = zeros(3, N + horizon_length - 1);
n_setpoint = size(setpoint, 2);
for k=1:n_setpoint
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 300)
        setpoint(:,k) = [10; 5; deg2rad(45)];
    else
        % setpoint(:,k) = [-5; -10; deg2rad(270)];
        setpoint(:,k) = [10; 5; deg2rad(45)];
    end
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
