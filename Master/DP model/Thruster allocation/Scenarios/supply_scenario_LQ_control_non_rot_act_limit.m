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

% Define thruster configuration matrix
% First column: Main propeller port
% Second column: Main propeller starboard
% Third column: Bow tunnel
main_propeller_y_distance = 3;
thruster_x_distance = 30;
T_conf = [1, 1, 0;
          0, 0, 1;
          main_propeller_y_distance, -main_propeller_y_distance, thruster_x_distance];

% Define force coefficient matrix
propeller_port_force_coefficient = 1000;
propeller_starboard_force_coefficient = propeller_port_force_coefficient;
bow_thruster_force_coefficient = 500;
K_force = diag([propeller_port_force_coefficient, propeller_starboard_force_coefficient, ...
    bow_thruster_force_coefficient]);

% Force weight
W_thr = diag([1, 1, 1]);

% Slack variable weight (punish deviation between requested and actual force)
Q_thr = diag([1e8, 1e8, 1e8]);

r_dim = 3;
n_dim = 3;

Phi_quad = [
                W_thr, zeros(r_dim, n_dim + 1);
                zeros(n_dim, r_dim), Q_thr, zeros(n_dim, 1);
                zeros(1,r_dim + n_dim + 1);
           ];

R_quad = zeros(r_dim + n_dim + 1, n_dim + r_dim + r_dim + 1);
R_quad(end, end) = 1;

A1 = [T_conf, -eye(n_dim), zeros(n_dim, 1)];

C1 = [eye(n_dim), zeros(n_dim, r_dim + r_dim + 1)];

A2 = [
        -eye(r_dim, r_dim), zeros(r_dim, n_dim + 1);
        eye(r_dim, r_dim), zeros(r_dim, n_dim + 1);
        -eye(r_dim, r_dim), zeros(r_dim, n_dim), -ones(r_dim, 1);
        eye(r_dim, r_dim), zeros(r_dim, n_dim), -ones(r_dim, 1);
     ];

C2 = [
        zeros(r_dim, n_dim), -eye(r_dim, r_dim), zeros(r_dim, r_dim + 1);
        zeros(r_dim, n_dim + r_dim), eye(r_dim, r_dim), zeros(r_dim, 1);
        zeros(r_dim, n_dim + r_dim + r_dim + 1);
        zeros(r_dim, n_dim + r_dim + r_dim + 1);
     ];

% Max and minimum force per thruster
fmax = [1e8; 1e8; 1e10];
fmin = -[1e8; 1e8; 1e10];

% Beta (punishing large force values)
beta = 1e0;

% Is there a linear or quadratic relation between force and RPM (or pitch angle) 
linear_force_rpm_relation = true;

% Optimizaton options
options = optimoptions('quadprog','Display','off');
