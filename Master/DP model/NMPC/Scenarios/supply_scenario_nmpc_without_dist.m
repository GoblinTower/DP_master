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

% Output files
folder = "Results/nmpc_no_dist";      % Name of folder to store output files
file_prefix = "nmpc_no_dist_";        % Prefix of file names

% MPC control parameters
horizon_length = 15;                 % Prediction horizon length
% Q = diag([1e14, 1e14, 1e16]);      % Error weighting matrix
Q = diag([1e11, 1e11, 1e11]);        % Error weighting matrix
% P = 1.0*diag([1, 1, 1]);           % Input weighting matrix
P = 1.0*diag([1, 1, 0.00001]);       % Input weighting matrix

options = optimoptions('fmincon', 'display', 'off');
% options = optimoptions('fmincon', 'display', 'iter','Algorithm','sqp');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Setpoints [North, East, Yaw] %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setpoint = zeros(3, N + horizon_length - 1);
n_setpoint = size(setpoint, 2);
for k=1:n_setpoint
    time = k*dt;
    if (time < 25)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 300)
        setpoint(:,k) = [10; 1; deg2rad(90)];
        % setpoint(:,k) = [10; 0; deg2rad(90)];
    else
        setpoint(:,k) = [-5; -10; deg2rad(270)];
        % setpoint(:,k) = [0; 0; deg2rad(270)];
    end
end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

V = diag([1;1;1]);                          % Process noise
W = diag([1;1;1;1;1;1;1e10;1e10;1e12]);     % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];       % Initial state estimate
n_kal_dim = size(x0_est,1);                 % Number of states in Kalman filter
G_lin = eye(n_kal_dim);                     % Process noise matrix

% p_aposteriori = 1.0*eye(9);                 % Aposteriori covariance estimate
x_aposteriori = x0_est;                     % Aposteriori state estimate

animate_kalman_estimate = true;             % Animate kalman estimate
animation_delay = 0;                        % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
% y = [x, y, psi]
% x = [x, y, psi, u, v, r]

x0 = [0; 0; 0; 0; 0; 0];                % Initial values of states (real)
n_dim = size(x0,1);                     % Size of state vector in process model

y0_meas = x0(1:3);                      % Initial values of measurements

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurement noise %%%
%%%%%%%%%%%%%%%%%%%%%%%%%
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter of the linearized model
% The vector represents measurement error in north position, east position
% and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.2; 0.2; 0.1];

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = false;
use_wave_force = false;
use_wind_force = false;

% Current
current_variance = [1e3; 1e3; 0];
% current_variance = [0; 0; 0];
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
% wave_variance = [0; 0; 0];
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
