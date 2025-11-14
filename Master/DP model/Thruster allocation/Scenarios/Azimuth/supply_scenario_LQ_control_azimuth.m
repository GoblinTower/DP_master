% Configuration script: LQ optimal control of supply model

dt = 1.0;           % Timestep used in integration

T = 1000;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/lq_azimuth_dist";                       % Name of folder to store output files
file_prefix = "lq_azimuth_dist_";                         % Prefix of file names
workspace_file_name = 'lq_azimuth_dist_data';             % Name of workspace file

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
        setpoint(:,k) = [10; 5; deg2rad(30)];
    else
        setpoint(:,k) = [0; -5; deg2rad(45)];
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

animate_kalman_estimate = true; % Animate kalman estimate
animation_delay = 0.25;         % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
% x = [x, y, psi, u, v, r]
% y = [x, y, psi]

% x0 = [0; 0; 0; 0; 0; 0];                        % Initial values of states (real)
x0 = [0; 0; 0; 0; 0; 0];                          % Initial values of states (real)
n_dim = size(x0,1);                               % Size of state vector in process model

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Thruster configuration matrix %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First column: Main propeller port
% Second column: Main propeller starboard
% Third column: Bow azimuth
% Fourth column: Aft azimuth
propeller_port_force_coefficient = 1000;
propeller_starboard_force_coefficient = propeller_port_force_coefficient;
azimuth_thruster_force_coefficient = 500;
K_force = diag([propeller_port_force_coefficient, propeller_starboard_force_coefficient, ...
    azimuth_thruster_force_coefficient, azimuth_thruster_force_coefficient]);

% Minimum and maximum force per thruster
fmin = -[1e8; 1e8; 1e10; 1e10];
fmax = [1e8; 1e8; 1e10; 1e10];

% alpha angle azimuths
% Here it is assumed that there are no limitations on turning
max_turn = deg2rad(1e5);
alpha_min = [-max_turn; -max_turn];
alpha_max = [max_turn; max_turn];
% alpha_min = [-deg2rad(180); deg2rad(180)];
% alpha_max = [-deg2rad(180); deg2rad(180)];

% maximum allowed alpha change between timesteps
alpha_diff_min = [deg2rad(-10); deg2rad(-10)];
alpha_diff_max = [deg2rad(10); deg2rad(10)];

% Force weight
W_thr = diag([1, 1, 1, 1]);

% Slack variable weight (punish deviation between requested and actual force)
Q_thr = diag([1e8,  1e8, 1e8]);

% Azimuth angle change weight
Omega_thr = diag([1, 1]);

% Power consumption weights
P_thr = [1e4; 1e4; 1e4; 1e4];
% P_thr = [0; 0; 0; 0];

% scalar weight
rho = 1e3;

epsilon = 1e-6; % Avoid divding by zero  

% Optimizaton options
% options = optimoptions('fmincon', 'Algorithm', 'sqp', 'display', 'off');
options = optimoptions('fmincon', 'display', 'off');

% 1: Port main propeller
% 2: Stern main propeller
% 3: Bow Azimuth thruster
% 4: Stern Azimuth thruster
thruster_positions = [
                        -35, -35, 30, -20;      % Placement along surge axis 
                        -3, 3, 0, 0             % Placement along sway axis
                     ];

thruster_names = ["PortMainProp1", "StarboardMainProp2", "BowAzimuth", "AftAzimuth"];

n_thrusters = size(thruster_positions,2);               % Number of thrusters
n_azimuths = 2;                                         % Number of azimuths
n_slack_variables = 3;                                  % Number of slack variables

z_dim = n_thrusters + n_azimuths + n_slack_variables;   % Number of optimization variables

angle_names = ["BowAzimuth", "AftAzimuth"];
alpha0 = [deg2rad(0); deg2rad(0)];                      % Azimuths starts in zero position

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = true;
use_wave_force = true;
use_wind_force = true;

run '..\common_external_disturbances.m';
