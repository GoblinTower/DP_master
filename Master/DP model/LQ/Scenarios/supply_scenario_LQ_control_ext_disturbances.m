% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The setpoint is maintained by using LQ
% optimal control strategy. Wind and current forces are introduced.

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
Q = diag([1e9, 1e9, 1e11]);          % State weighting matrix
R = 1.0*eye(3);                      % Input weighting matrix

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
run_kalman_filter = false;

V = 1.0*eye(9);            % Process noise
W = 1.0*eye(3);            % Measurement noise
X_apriori = 1.0*eye(9);    % Apriori estimate covariance

% Supply model
% x = [x, y, psi, u, v, r]
x0 = [0; 0; 0; 0; 0; 0];                % Inital values of states

% Linearized model with integral gain
% x = [x, y, psi, u, v, r, b1, b2, b3]
x_lin0 = [0; 0; 0; 0; 0; 0];            % Inital values of states

% Measurement noise
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.2; 0.2; deg2rad(0.1)];

use_wind_current_forces = true;
if (use_wind_current_forces)
    % wind_force = [1e5; 3e5; 1e6].*ones(3,N) + normrnd(0,0.1,3,N);       % Wind
    current_force = [2e5; -3e5; 2e6].*ones(3,N) + normrnd(0,0.1,3,N);   % Current
else
    % wind_force = zeros(3,N);                                            % Wind
    current_force = zeros(3,N);                                         % Current
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
variance = [0.1; 0.2]*dt;
start_values = [deg2rad(35); 10];

% Gaussian random walk
wind = zeros(2,N);
wind(:,1) = [start_values(1), start_values(2)];

for j=2:N
    wind(:,j) = wind(:,j-1) + normrnd(0, variance, 2, 1);
end

wind_beta = smooth(wind(1,:));
wind_abs = smooth(wind(2,:));

% figure(1);
% subplot(4,1,1);
% plot(wind(1,:));
% title("Angle of attack (beta)")
% subplot(4,1,2);
% plot(wind(2,:));
% title("Wind speed (absolute value)")
% subplot(4,1,3);
% plot(wind_beta);
% title("Angle of attack (smoothed))")
% subplot(4,1,4);
% plot(wind_abs);
% title("Wind speed (smoothed)")
