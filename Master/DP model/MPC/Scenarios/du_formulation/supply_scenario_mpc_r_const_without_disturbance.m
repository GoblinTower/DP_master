% Implementation of MPC strategy to maintain supply boat position and
% heading. This script assumes the heading rate of change remains constant
% during the prediction horizon of the MPC. 

dt = 1.0;           % Timestep used in integration

T = 900;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/mpc_const_r_du_without_dist";                   % Name of folder to store output files
file_prefix = "mpc_const_r_du_without_dist_";                     % Prefix of file names
workspace_file_name = "mpc_const_r_du_without_dist.mat";          % Name of .mat file

store_workspace = true;                                           % Flag to indicate whether to save workspace to mat file

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MPC control parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
horizon_length = 20;                 % Prediction horizon length

% Q = diag([5e8, 5e8, 5e8]);           % Error weighting matrix
% P = diag([1e-4, 1e-4, 1e-6]);        % Input weighting matrix
Q = diag([1e8, 1e8, 5e8]);           % Error weighting matrix
P = diag([1e-4, 1e-4, 1e-6]);        % Input weighting matrix

% Quadratic programming options
% options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'display', 'off');
options = optimoptions('quadprog', 'display', 'off');

% Force and momentum limitations
use_force_limitation = false;        % Apply force and momentum limitations
max_inputs = [1e10; 1e10; 1e10];     % Max allowed force/momentum
max_delta_u = [1e6; 1e6; 2e7];       % Max allowed change in force/momentum per timestep 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Setpoints [North, East, Yaw] %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setpoint_length = N+horizon_length-1;
setpoint = zeros(3, setpoint_length);
for k=1:setpoint_length
    time = k*dt;
    if (time < 300)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 600)
        setpoint(:,k) = [10; 5; deg2rad(26)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(-225)];
    end
end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

W = diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e12, 1e12, 1e12]);   % Process noise
V = 10*eye(3);                                                % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0];             % Initial state estimate
n_kal_dim = size(x0_est,1);                       % Number of states in Kalman filter
G_lin = eye(n_kal_dim);                           % Process noise matrix

x_aposteriori = x0_est;                           % Aposteriori state estimate

animate_kalman_estimate = true;                   % Animate kalman estimate
animation_delay = 0.01;                           % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
% x = [x, y, psi, u, v, r]
% y = [x, y, psi]

x0 = [0; 0; 0; 0; 0; 0];                          % Initial values of states (real)
% x0 = [0.5; 0.5; deg2rad(2); 0; 0; 0];           % Initial values of states (real)
n_dim = size(x0,1);                               % Size of state matrix in process model

y0_meas = x0(1:3);                                % Initial values of measurements
m_dim = size(y0_meas,1);                          % Size of output vector in process model

r_dim = 3;                                        % Sice of input vector

u_prev = zeros(r_dim,1);                          % Previous input vector. Needed for first iteration

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
