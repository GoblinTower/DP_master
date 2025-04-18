% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The control signal is calculated
% using a non-linear MPC.

dt = 1.0;           % Timestep used in integration
T = 100;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler

% integration_method = IntegrationMethod.Forward_Euler;
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% MPC control parameters
horizon_length = 20;                 % Prediction horizon length
% Q = diag([1e14, 1e14, 1e16]);        % Error weighting matrix
Q = diag([1e14, 1e14, 1e18]);        % Error weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

options = optimoptions('fmincon', 'display', 'off');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

% Setpoints [North, East, Yaw]
setpoint = zeros(3, N + horizon_length - 1);
n_setpoint = size(setpoint, 2);
for k=1:n_setpoint
    time = k*dt;
    if (time < 25)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 300)
        setpoint(:,k) = [10; 5; deg2rad(90)];
    else
        setpoint(:,k) = [-5; -10; deg2rad(270)];
    end
end

% Kalman filter
run_kalman_filter = true;

V = diag([1;1;1]);                          % Process noise
W = diag([1;1;1;1;1;1;1e10;1e10;1e12]);     % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 1e4; 1e4; 1e4]; % Initial state estimate

p_aposteriori = 1.0*eye(9);                 % Aposteriori covariance estimate
x_aposteriori = x0_est;                     % Aposteriori state estimate

animate_kalman_estimate = true;             % Animate kalman estimate
animation_delay = 0;                        % Animation speed (in seconds)

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
measurement_noise_std = [0.2; 0.2; 0.1];

use_wind_forces = false;
use_wave_forces = false;
use_current_forces = true;

if (use_wind_forces)
    wind_forces = [5e4; 2e5; 1e5].*ones(3,N);
else
    wind_forces = zeros(3,N);
end

if (use_wave_forces)
    wave_forces = [5e4; 1e5; 0].*ones(3,N); 
else
    wave_forces = zeros(3,N);
end

if (use_current_forces)
    current_forces = [1e5; 0; 0].*ones(3,N);
else
    current_forces = zeros(3,N);                                                    
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
std = [0.1; 0.2];
mean = [0; 0];
start_values = [deg2rad(35); 10];

% Gaussian random walk
wind = zeros(2,N);
wind(:,1) = [start_values(1); start_values(2)];
for j=2:N
    wind(:,j) = wind(:,j-1) + normrnd(mean, std, 2, 1);
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
