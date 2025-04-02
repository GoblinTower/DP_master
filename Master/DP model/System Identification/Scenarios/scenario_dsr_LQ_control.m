% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The setpoint is maintained by using LQ
% optimal control strategy.

dt = 1.0;           % Timestep used in integration

T = 500;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% LQ control parameters
Q = diag([1e8, 1e8, 1e8]);           % Error weighting matrix
R = 1.0*eye(4);                      % Input weighting matrix

% Constant azimuth angles
azimuth_angle_1 = 180;
azimuth_angle_2 = 180;

% Setpoints [North, East, Yaw]
setpoint = zeros(3,N+1);
for k=1:N
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 1000)
        setpoint(:,k) = [1; 1; deg2rad(0)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(90)];
    end
end

% Kalman filter
run_kalman_filter = false;

V = 1.0*eye(6);            % Process noise
W = 1.0*eye(3);            % Measurement noise
X_apriori = 1.0*eye(9);    % Apriori estimate covariance

% Starting position
% x = [x, y, psi, u, v, r]
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Inital values of states
u0 = [0; 0; 0; 0];                               % Initial values of inputs

% Linearized model with integral gain
% x = [x, y, psi, u, v, r, b1, b2, b3]
x_lin0 = [0; 0; 0; 0; 0; 0];                     % Inital values of states

% Measurement noise
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.2; 0.2; deg2rad(0.1)];
