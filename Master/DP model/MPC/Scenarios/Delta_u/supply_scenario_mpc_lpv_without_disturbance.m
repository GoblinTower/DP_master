% Implementation of MPC strategy to maintain supply boat position and
% heading. This script assumes the heading rate of change remains constant
% during the prediction horizon of the MPC. 

% Add seed (meaning og life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 500;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/mpc_lpv_du_no_dist";         % Name of folder to store output files
file_prefix = "mpc_lpv_du_no_dist";            % Prefix of file names

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MPC control parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
horizon_length = 20;                 % Prediction horizon length
% Q = diag([1e8, 1e8, 1e10]);        % Error weighting matrix
% P = diag([1e-6, 1e-6, 1e-6]);      % Input weighting matrix

Q = diag([5e8, 5e8, 5e9]);           % Error weighting matrix
P = diag([1e-4, 1e-4, 1e-6]);        % Input weighting matrix

% Quadratic programming options
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
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 400)
        setpoint(:,k) = [10; 5; deg2rad(30)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(45)];
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
