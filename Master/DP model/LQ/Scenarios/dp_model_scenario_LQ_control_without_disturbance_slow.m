% Configuration script: LQ optimal control using supply model
% as process and DP model for control and filtering.

% Add seed (meaning of life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 1000;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/lq_dp_model_no_dist";                % Name of folder to store output files
file_prefix = "lq_dp_model_no_dist_";                  % Prefix of file names
workspace_file_name = 'lq_dp_model_no_dist_data';      % Name of workspace file

store_workspace = true;

% LQ control parameters
Q = diag([1e9, 1e9, 1e11]);          % State weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

% Setpoints [North, East, Yaw]
setpoint = zeros(3,N+1);
for k=1:N+1
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 600)
        setpoint(:,k) = [10; 5; deg2rad(26)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(225)];
    end
end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

W = diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e12, 1e12, 1e12]);   % Process noise
V = 10*eye(3);                                                % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];             % Initial state estimate
n_kal_dim = size(x0_est,1);                       % Size of state matrix in Kalman filter
G_lin = eye(n_kal_dim);                           % Process noise matrix

x_aposteriori = x0_est;                           % Aposteriori state estimate

animate_kalman_estimate = true;                   % Animate kalman estimate
animation_delay = 0.25;                           % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
% x = [x, y, psi, u, v, r]
% y = [x, y, psi]

% x0 = [0; 0; 0; 0; 0; 0];                        % Initial values of states (real)
x0 = [0; 0; 0; 0; 0; 0];                          % Initial values of states (real)
n_dim = size(x0,1);                               % Size of state matrix in process model

y0_meas = x0(1:3);                                % Initial values of measurements

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurement noise %%%
%%%%%%%%%%%%%%%%%%%%%%%%%
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter. The vector represents measurement 
% error in north position, east position and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = false;
use_wave_force = false;
use_wind_force = false;

run 'common_external_disturbances.m';
