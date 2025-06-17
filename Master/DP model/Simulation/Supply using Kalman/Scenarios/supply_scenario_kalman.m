% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. Kalman gain is calculated.

dt = 1.0;           % Timestep used in integration

T = 500;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% LQ control parameters
Q = diag([1e6, 1e6, 5e8]);           % Error weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

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

% Generate predefined control inputs using Gaussian random walk
% u is generalized force, hence it must be allocated to each of the
% thrusters/propellers/azimuths/rudders
u = zeros(3,N);
u0 = [5e4; 5e4; 0];

variance = [2e3; 2e3; 2e4]*dt;          % Thruster forces in surge, sway and momentum in jaw

% Gaussian random walk
u(:,1) = u0;
for j=2:N
    u(:,j) = u(:,j-1) + normrnd(0, variance, 3, 1);
end
