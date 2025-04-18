% Nonlinear MPC applied to supply model.
% Two models can be used in the model based comparison in MPC:
% DP model by Fossen and reduced DP model (without the error term, b).
% Here the complete model of Fossen is used for control.
% This script uses a linear  Kalman filter (assuming vessel heading is known).
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
% run 'Scenarios'\supply_scenario_non_linear_mpc_control_lin_kalman_tracking.m;
run 'Scenarios'\supply_scenario_non_linear_mpc_control_lin_kalman_tracking_lim.m;

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1, N+1);        % Time array

x_array = zeros(6,N+1);         % State array from real process (unknown)
x_array(:,1) = x0;              % Storing initial value of state

x_est_array = zeros(9,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;      % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);    % Measurement array
y_meas_array(:,1) = y0_meas;    % Storing initial value of measurement 

u_array = zeros(3,N);           % Control input array

setpoint_array = zeros(3,N);    % Array of setpoints

% Initial values
x = x0;                         % Initial real state 
x_est = x0_est;                 % Initial state estimate
y_meas = y0_meas;               % Initial measured value         

t = 0;                          % Current time

% Create linear kalman filter
kalman = LinearKalmanFilter(x_aposteriori, p_aposteriori);

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros(9*3,N);         % Storing Kalman filter gain

% Current waypoint index
waypoint_index = 1;                                 % Waypoint index (for keeping track of current waypoint)
ref = waypoints(:,1).*ones(3, horizon_length);  % The first waypoint is set as setpoint

for i=1:N

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate control signal u %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Find current waypoint
    % Change tracking when distance to next waypoint is less than some
    % specified distance
    if (norm(waypoints(1:2,waypoint_index) - x_est(1:2)) < distance_to_update_setpoint ...
            && waypoint_index ~= last_waypoint_index)
        waypoint_index = waypoint_index + 1;
        ref = waypoints(:,waypoint_index).*ones(3, horizon_length);
    end

    % Solve non-linear optimization problem
    if (use_thruster_constraints)
        previous_control_signal = u0(:,1);
        [Ai, bi] = calculate_thruster_inequality_matrix(horizon_length, z_dim, previous_control_signal, max_inputs, max_delta_u);
        u_sol = fmincon(@(u) non_linear_objective_function(u, ref, M, D, P, Q, x_est, horizon_length, dt, 1), u0, Ai, bi, [], [], [], [], [], options);
    else
        u_sol = fmincon(@(u) non_linear_objective_function(u, ref, M, D, P, Q, x_est, horizon_length, dt, 1), u0, [], [], [], [], [], [], [], options);
    end

    % Store old value of optimal control input for initial guess in next
    % iteration (warm starting)
    u0 = u_sol;

    % Get control signal
    u = u_sol(:,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update model (Real process) %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch (integration_method)
        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            xdot = supply(x,u);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply(x, u), t, x, dt);
    end

    % Measurement with added noise
    if (use_noise_in_measurements)
        y_meas = x(1:3) + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = x(1:3);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
 
    % Calculate vessel heading
    psi = y_meas_array(3,i);

    % Calculate discrete supply model matrices
    [Ad, Bd, Cd] = dp_fossen_discrete_matrices(M, D, psi, dt, false);

    % Calculate kalman gain
    [x_est, Pcov, K] = kalman.UpdateFilter(u, y_meas_array(:,i), Ad, Bd, Cd, W, V);

    % Store Kalman gain
    K_array(:,i) = K(:);

    % Update time
    t = t + dt;
    disp(['Current time: ', num2str(t)]);
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    x_est_array(:,i+1) = x_est;
    y_meas_array(:,i+1) = y_meas;
    u_array(:,i) = u;
    setpoint_array(:,i) = ref(:,1);

    % Update animated positon plot
    if (animate_kalman_estimate)
        animate_kalman.UpdatePlot(t_array(i), x_est_array(1,i), x_est_array(2,i), x_est_array(3,i),...
            y_meas_array(1,i), y_meas_array(2,i), y_meas_array(3,i),...
            setpoint_array(1,i), setpoint_array(2,i), setpoint_array(3,i));
        
        pause(animation_delay);
    end
    
end

% Plot data
plot_supply_nmpc_no_disturbance_tracking(t_array, x_array, K_array, u_array, setpoint_array, waypoints, true);
