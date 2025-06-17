% Configuration script: LQ optimal control using supply model
% as process and DP model for control and filtering.

% Add seed (meaning of life)
rng(42,"twister");

dt = 1.0;           % Timestep used in integration

T = 1600;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% Output files
folder = "Results/tracking_lq_dist";        % Name of folder to store output files
file_prefix = "lq_dist_tracking_";          % Prefix of file names

% LQ control parameters
Q = diag([1e11, 1e11, 1e13]);          % State weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

% Setpoints [North, East, Yaw]
% setpoint = zeros(3,N+1);
% for k=1:N+1
%     time = k*dt;
%     if (time < 100)
%         setpoint(:,k) = [0; 0; 0];
%     elseif (time < 400)
%         setpoint(:,k) = [10; 5; deg2rad(30)];
%     else
%         setpoint(:,k) = [-5; -5; deg2rad(45)];
%     end
% end

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

W = diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e12, 1e12, 1e12]);   % Process noise
V = 10*eye(3);                                                % Measurement noise

x0_est = [0; 0; 0.4869; 0; 0; 0; 0; 0; 0];             % Initial state estimate
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

% x0 = [0; 0; 0; 0; 0; 0];                        % Initial values of states (real)
x0 = [0; 0; 0.4869; 0; 0; 0];                     % Initial values of states (real)
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

%%%%%%%%%%%%%%%%%%
%%% Trajectory %%%
%%%%%%%%%%%%%%%%%%
% Creating waypoints
t = [0, 200, 400, 600, 800, 1000, 1200, 1400, 1600];
x = [0, 10, 20, 30, 40, -10, -2, -20, -20];
y = [0, 5, 15, 10, 10, 35, 60, 80, 80];

delta_t = t(2:end) - t(1:end-1);
delta_x = x(2:end) - x(1:end-1);
delta_y = y(2:end) - y(1:end-1);

xdot = [delta_x./delta_t, 0.0];
ydot = [delta_y./delta_t, 0.0];

% Calling trajectory function
[t_c, x_c, y_c, xdot_c, ydot_c] = cubic_interpolation(t, x, y, xdot, ydot, dt);

% Calculate yaw angle
diff_x = x_c(2:end) - x_c(1:end-1);
diff_y = y_c(2:end) - y_c(1:end-1);
yaw_setpoints = [atan2(diff_y, diff_x), atan2(diff_y(end), diff_x(end))];

setpoint = [x_c; y_c; yaw_setpoints];

%%%%%%%%%%%%%%%%%%%%%%%
%%% External forces %%%
%%%%%%%%%%%%%%%%%%%%%%%
use_current_force = true;
use_wave_force = true;
use_wind_force = true;

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
