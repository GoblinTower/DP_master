% Configuration script: LQ optimal control of OSV model

% Add seed (meaning og life)
rng(42,"twister");

dt = 0.3;           % Timestep used in integration

T = 1000;           % End time
N = ceil(T/dt);     % Number of sample steps

n_dim_control = 9;

% Simulation type:
% '_no_dist' or '_dist'
simulation_type = '_dist';

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

% LQ control parameters
Q = diag([1e9, 1e9, 1e11]);          
% Q = diag([1e5, 1e5, 1e8]);           % State weighting matrix
P = 1.0*eye(3);                      % Input weighting matrix

% Setpoints [North, East, Yaw]
setpoint = zeros(3,N+1);
for k=1:N
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];
    elseif (time < 600)
        setpoint(:,k) = [10; 5; deg2rad(30)];
    else
        setpoint(:,k) = [-5; -5; deg2rad(45)];
    end
end

% Constant azimuth angles
azimuth_angle_1 = 0;
azimuth_angle_2 = 0;

%%%%%%%%%%%%%%%%%%%%%
%%% Kalman filter %%%
%%%%%%%%%%%%%%%%%%%%%
run_kalman_filter = true;

W = diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e12, 1e12, 1e12]);      % Process noise
V = 10.0*eye(3);                                                                % Measurement noise

x0_est = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];    % Initial state estimate
n_kal_dim = size(x0_est,1);                       % Number of states in Kalman filter
G_lin = eye(n_kal_dim);                           % Process noise matrix

x_aposteriori = x0_est;                           % Aposteriori state estimate

animate_kalman_estimate = true;                   % Animate kalman estimate
animation_delay = 0.01;                           % Animation speed (in seconds)

%%%%%%%%%%%%%%%%%%%%%
%%% Process model %%%
%%%%%%%%%%%%%%%%%%%%%
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Initial values of states (real)
n_dim = size(x0,1);                              % Size of state matrix in process model

y0_meas = x0(1:3);                               % Initial values of measurements

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurement noise %%%
%%%%%%%%%%%%%%%%%%%%%%%%%
% This represent the noise added to the measurement vector from the supply
% model before being fed to the Kalman filter. The vector represents measurement 
% error in north position, east position and yaw position respectively.
use_noise_in_measurements = false;

measurement_noise_mean = [0; 0; 0];
measurement_noise_std = [0.1; 0.1; deg2rad(0.1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% External disturbances %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
use_current_velocity = true;

% Current velocity
current_velocity = zeros(1,N);
current_vel_variance = 0.02;
current_vel_start_values = 1.0;

% Current angle
current_angle = zeros(1,N);
current_angle_variance = deg2rad(0.05);
current_angle_start_values = deg2rad(0);

if (use_current_velocity)
    current_velocity(1) = current_vel_start_values;
    current_angle(1) = current_angle_start_values;
    for j=2:N

        current_velocity(j) = current_velocity(j-1) + normrnd(0, current_vel_variance, 1);
        current_angle(j) = current_angle(j-1) + normrnd(0, current_angle_variance, 1);
        
        % Velocity not allowed faster than 3 m/s
        if (current_velocity(j) > 3)
            current_velocity(j) = 3;
        end
        if (current_velocity(j) < -3)
            current_velocity(j) = -3;
        end
    end
end