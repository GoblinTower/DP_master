% A linearized model is used for comparison with the supply model
% developed by Thor Inge Fossen. The control signal is calculated
% using a non-linear MPC.

dt = 1.0;           % Timestep used in integration

T = 200;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler

% integration_method = IntegrationMethod.Forward_Euler;
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/nmpc_no_green_dp_dist_drift";                % Name of folder to store output files
file_prefix = "nmpc_no_green_dp_dist_drift_";                  % Prefix of file names
workspace_file_name = "nmpc_no_green_dp_dist_drift_data";      % File name of .mat file

store_workspace = true;                                        % Should the workspace be stored after running the simulation?

% MPC control parameters
horizon_length = 20;                 % Prediction horizon length
groups = [2,8,10];                   % Grouping

% Q = diag([1e11, 1e11, 1e12]);        % Error weighting matrix
% P = 1.0*diag([1, 1, 1e-4]);          % Input weighting matrix
Q = diag([5e8, 5e8, 1e9]);           % Error weighting matrix
P = diag([1e-4, 1e-4, 1e-6]);        % Input weighting matrix

% options = optimoptions('fmincon', 'display', 'iter','Algorithm','sqp');
options = optimoptions('fmincon', 'display', 'off');

% Initial guess of control signal for non-linear optimization algorithm
u0 = zeros(3,horizon_length);

%%%%%%%%%%%%%%%%
%%% Green DP %%%
%%%%%%%%%%%%%%%%
run_green_dp = false;

W1 = 1e2;               % Green DP weight (1)
W2 = 1e8;               % Green DP weight (2)
R1 = 5;                 % Distance to border

P_green = eye(3);       % Weighting of input

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Setpoints [North, East, Yaw] %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setpoint = zeros(3, N + horizon_length - 1);
n_setpoint = size(setpoint, 2);
for k=1:n_setpoint
    time = k*dt;

    setpoint(:,k) = [0; 0; 0];

end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

V = diag([1;1;1]);                          % Process noise
W = diag([1;1;1;1;1;1;1e12;1e12;1e12]);     % Measurement noise

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
use_current_force = true;
use_wave_force = true;
use_wind_force = true;

run 'common_external_disturbances_drift.m';
