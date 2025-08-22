% Nonlinear MPC applied to supply model.
% Two models can be used in the model based comparison in MPC:
% DP model by Fossen and reduced DP model (without the error term, b).
% Here the complete model of Fossen is used for control.
% This script uses a linear  Kalman filter (assuming vessel heading is known).
% It is assumed that ship is exposed to external forces.
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data

% Circle scenario
% run 'Scenarios\Circle\supply_scenario_nmpc_with_dist_no_green_dp_circle';
% run 'Scenarios\Circle\supply_scenario_nmpc_with_dist_green_dp_circle';

% Drift
% run 'Scenarios\Drift\supply_scenario_nmpc_with_dist_no_green_dp_drift';
% run 'Scenarios\Drift\supply_scenario_nmpc_with_dist_green_dp_drift';

% Mild (small weather disturbances)
% run 'Scenarios\Mild\supply_scenario_nmpc_with_dist_no_green_dp_mild';
% run 'Scenarios\Mild\supply_scenario_nmpc_with_dist_green_dp_mild';

% Oscillations
run 'Scenarios\Oscillations\supply_scenario_nmpc_with_dist_no_green_dp_oscillations';
% run 'Scenarios\Oscillations\supply_scenario_nmpc_with_dist_green_dp_oscillations';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by Thor
% Inge Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1, N+1);                % Time array

wind_force_array = zeros(3,N);          % Wind force array relative to BODY coordinate frame
wave_force_array = zeros(3,N);          % Wave force array relative to BODY coordinate frame
current_force_array = zeros(3,N);       % Current force array relative to BODY coordinate frame

x_array = zeros(n_dim,N+1);             % State array from real process (unknown)
x_array(:,1) = x0;                      % Storing initial value of state

x_est_array = zeros(n_kal_dim,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;              % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);            % Measurement array
y_meas_array(:,1) = y0_meas;            % Storing initial value of measurement 

u_array = zeros(3,N);                   % Control input array

% Initial values
x = x0;                                 % Initial real state 
x_est = x0_est;                         % Initial state estimate
y_meas = y0_meas;                       % Initial measured value         

t = 0;                                  % Current time

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros(9*3,N);                 % Storing Kalman filter gain

for i=1:N

    % Get vessel heading
    psi = y_meas(3);

    % Get wind forces and momentum
    % The actual wind forces will depend on the real ship position, hence
    % we use the state variables from the 'real' process.
    wind_force = wind_force_calc(wind_abs(i), wind_beta(i), x(3), x(4), x(5), rho, Af, Al, L, Cx, Cy, Cn);
    if (use_wind_force)
        wind_force_array(:,i) = wind_force;
    else
        wind_force_array(:,i) = zeros(3,1);
    end

    % Wave force is defined relative to NED coordinate frame. Forces must
    % transformed to BODY Coordinate frame
    rot = rotation_matrix(psi);
    wave_force_array(:,i) = rot'*wave_force(:,i);

    % Current force is defined relative to NED coordinate frame. Forces must
    % transformed to BODY Coordinate frame
    current_force_array(:,i) = rot'*current_force(:,i);

    % Known forces, tau
    tau = wind_force_array(:,i) + wave_force_array(:,i);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate control signal u %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ref = setpoint(:,i:(i+horizon_length-1));

    % Solve non-linear optimization problem
    if (run_kalman_filter)
        % Use state from Kalman filter
        if (run_green_dp)
             u_sol = fmincon(@(u) non_linear_green_dp_objective_function(u, tau, ref, M, D, P_green, R1, W1, W2, x_est, horizon_length, dt, 1), u0, [], [], [], [], [], [], @(u) grouping(u, groups), options);
        else
             u_sol = fmincon(@(u) non_linear_objective_du_function(u, tau, ref, M, D, P, Q, x_est, u0(:,1), horizon_length, dt, 1), u0, [], [], [], [], [], [], @(u) grouping(u, groups), options);
        end
    else
        % Use real state (assumed known, perfect information)
        % b is unknown, and hence set to zero
        x_extended = [x; 0; 0; 0];
        if (run_green_dp)
             u_sol = fmincon(@(u) non_linear_green_dp_objective_function(u, tau, ref, M, D, P_green, R1, W1, W2, x_extended, horizon_length, dt, 1), u0, [], [], [], [], [], [], @(u) grouping(u, groups), options);
        else
             u_sol = fmincon(@(u) non_linear_objective_du_function(u, tau, ref, M, D, P, Q, x_extended, horizon_length, dt, 1), u0, [], [], [], [], [], [], @(u) grouping(u, groups), options);
        end
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
            xdot = supply_model(t, x, u, M, D, wind_force_array(:,i), wave_force_array(:,i), current_force_array(:,i), t, x, dt);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind_force_array(:,i), wave_force_array(:,i), current_force_array(:,i)), t, x, dt);
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
    if (run_kalman_filter)

        % Calculate discrete dp model matrices
        [A_lin, B_lin, F_lin, C_lin] = dp_fossen_discrete_matrices(M, D, psi, dt, false);

        % Get Kalman gain from inbuilt MATLAB function dlqe
        [K,~,~,~] = dlqe(A_lin, G_lin, C_lin, W, V);

        % Calculate dx_apriori at time step (i+1)
        x_apriori = A_lin*x_aposteriori + B_lin*u + F_lin*tau;

        % Calculate dx_aposteriori at time step (i+1)
        x_aposteriori = x_apriori + K*(y_meas - C_lin*x_apriori);

        x_est = x_aposteriori;

        % Store data Kalman gain
        K_array(:,i) = K(:);

    end

    % Update time
    t = t + dt;

    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    x_est_array(:,i+1) = x_est;
    y_meas_array(:,i+1) = y_meas;
    u_array(:,i) = u;

    % Output data
    disp(['Current time: ', num2str(t)]);
    disp(['Integrator term : ', 'b(1): ', num2str(x_est(7)), ' b(2): ', num2str(x_est(8)), ...
        ' b(3): ', num2str(x_est(9))]);

    % Update animated positon plot
    if (animate_kalman_estimate)
        animate_kalman.UpdatePlot(t_array(i), x_est_array(1,i), x_est_array(2,i), x_est_array(3,i),...
            y_meas_array(1,i), y_meas_array(2,i), y_meas_array(3,i),...
            setpoint(1,i), setpoint(2,i), setpoint(3,i));
        
        pause(animation_delay);
    end
    
end

% Plot data
plot_supply_nmpc_green_dp(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, current_force, wave_force, setpoint, true, folder, file_prefix, R1, setpoint(1:2,1));

% Store workspace
if (store_workspace)
    % Create folder if it does not exists
    if (not(isfolder("Workspace")))
        mkdir("Workspace");
    end
    save("Workspace/" + workspace_file_name);
end