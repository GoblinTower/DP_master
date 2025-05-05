% Configuration script: LQ optimal control of supply model

% Add seed (meaning og life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 600;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% LQ control parameters
Q = diag([1e9, 1e9, 1e11]);          % State weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

% Setpoints [North, East, Yaw]
setpoint = zeros(3,N+1);
for k=1:N
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 300)
        setpoint(:,k) = [10; 5; deg2rad(30)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(45)];
    end
end

% Kalman filter
run_kalman_filter = true;

W = 1.0*eye(6);                 % Process noise
V = 10.0*eye(3);                % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0];    % Initial state estimate

p_aposteriori = 1.0*eye(6);     % Aposteriori covariance estimate
x_aposteriori = x0_est;         % Aposteriori state estimate

animate_kalman_estimate = true; % Animate kalman estimate
animation_delay = 0.01;         % Animation speed (in seconds)

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
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];

% Define force coefficient matrix
propeller_port_force_coefficient = 1000;
propeller_starboard_force_coefficient = propeller_port_force_coefficient;
azimuth_thruster_force_coefficient = 500;
K_force = diag([propeller_port_force_coefficient, propeller_starboard_force_coefficient, ...
    azimuth_thruster_force_coefficient, azimuth_thruster_force_coefficient]);

% Minimum and maximum force per thruster
fmin = -[1e8; 1e8; 1e10; 1e10];
fmax = [1e8; 1e8; 1e10; 1e10];

% fmin = -[1e20; 1e20; 1e20; 1e20];
% fmax = [1e20; 1e20; 1e20; 1e20];

% alpha angle azimuths
% Here it is assumed that there are no limitations on turning
max_turn = deg2rad(1e5);
alpha_min = [-max_turn; -max_turn];
alpha_max = [max_turn; max_turn];

% maximum allowed alpha change between timesteps
alpha_diff_min = [deg2rad(-10); deg2rad(-10)];
alpha_diff_max = [deg2rad(10); deg2rad(10)];

% Force weight
W_thr = diag([1, 1, 1, 1]);

% Slack variable weight (punish deviation between requested and actual force)
Q_thr = diag([1e6, 1e6, 1e6]);

% Azimuth angle change weight
Omega_thr = diag([1, 1]);

% Power consumption weights
P_thr = [1; 1; 1; 1];
P_thr = [0; 0; 0; 0];

% scalar weight
rho = 1.0;

epsilon = 1e-2; % Avoid divding by zero  

% Is there a linear or quadratic relation between force and RPM (or pitch angle) 
linear_force_rpm_relation = true;

% Optimizaton options
% options = optimoptions('fmincon', 'display', 'off');
