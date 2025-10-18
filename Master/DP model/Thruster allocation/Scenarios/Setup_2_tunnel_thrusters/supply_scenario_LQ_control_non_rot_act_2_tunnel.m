% Configuration script: LQ optimal control of supply model

dt = 1.0;           % Timestep used in integration

T = 1000;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/lq_nonrot_2tunnel_dist";                % Name of folder to store output files
file_prefix = "lq_nonrot_2tunnel_dist_";                  % Prefix of file names
workspace_file_name = 'lq_nonrot_2tunnel_dist_data';      % Name of workspace file

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
% Third column: Bow tunnel
main_propeller_port_y_distance = -3;
main_propeller_starboard_y_distance = 3;
thruster1_x_distance = -20;
thruster2_x_distance = 30;
T_conf = [1, 1, 0, 0;
          0, 0, 1, 1;
          -main_propeller_port_y_distance, -main_propeller_starboard_y_distance, thruster1_x_distance, thruster2_x_distance];

% Define force coefficient matrix
propeller_port_force_coefficient = 1000;
propeller_starboard_force_coefficient = propeller_port_force_coefficient;
bow_thruster_force_coefficient = 500;
K_force = diag([propeller_port_force_coefficient, propeller_starboard_force_coefficient, ...
    bow_thruster_force_coefficient, bow_thruster_force_coefficient]);

% Solving using Lagrange Multiplier, unconstrained
W_thr = diag([1,1,1,1]);                             % Weighting matrix

% 1: Port main propeller
% 2: Stern main propeller
% 3: Aft tunnel thruster
% 4: Bow tunnel thruster
thruster_positions = [
                        -35, -35, -20, 30;      % Placement along surge axis 
                        -3, 3, 0, 0             % Placement along sway axis
                     ];

thruster_names = ["PortMainProp1", "StarboardMainProp2", 'AftTunnelThruster', "BowTunnelThruster"];

thruster_angles = [deg2rad(0), deg2rad(0), deg2rad(90), deg2rad(90)];

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = true;
use_wave_force = true;
use_wind_force = true;

run '..\common_external_disturbances.m';
