% Script implementing LQ optimal control for the supply model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\Setup_1_tunnel_thruster\supply_scenario_LQ_control_non_rot_act_limit';
% run 'Scenarios\Setup_2_tunnel_thrusters\supply_scenario_LQ_control_non_rot_act_limit_2_tunnel';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1,N+1);                 % Time array

wind_force_array = zeros(3,N);          % Wind force array relative to BODY coordinate frame
wave_force_array = zeros(3,N);          % Wave force array relative to BODY coordinate frame
current_force_array = zeros(3,N);       % Current force array relative to BODY coordinate fram

x_array = zeros(n_dim,N+1);             % State array from real process (unknown)
x_array(:,1) = x0;                      % Storing initial value of state

x_est_array = zeros(n_kal_dim,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;              % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);            % Measurement array
y_meas_array(:,1) = y0_meas;            % Storing initial value of measurement 

u_array = zeros(3,N);                   % Control input array

u_prev = zeros(3,1);                    % Previous control signal

% Initial values
x_prev = zeros(n_dim,1);                % Previus state 
y_prev = zeros(3,1);                    % previous output
x = x0;                                 % Initial real state 
x_est = x0_est;                         % Initial state estimate
y_meas = y0_meas;                       % Initial measured value         

t = 0;                                  % Current time

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Number of thrusters
n_thrusters = size(T_conf,2);

% Thruster allocation
rpm_array = zeros(n_thrusters,N);       % Array of control signal to thruster (here defined as RPM)
f_array = zeros(n_thrusters,N);         % Array of individual thruster forces

% Store Kalman gain
K_array = zeros(n_kal_dim*3,N);   % Storing Kalman filter gain

% Thruster allocation plotting
animate_combined = AnimateCombined(70, 8, thruster_positions, thruster_names);

for i=1:N

    % Calculate vessel heading
    psi = y_meas(3);

    % Calculate discrete supply model matrices
    [A_lin, B_lin, C_lin] = supply_discrete_matrices(M, D, psi, dt, false);

    % Calculate LQ gain on deviation form
    [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P);

    % Control input using LQ
    ref = setpoint(:,i);
    if (run_kalman_filter)
        % Use state from Kalman filter
        u = u_prev + G1*(x_est(1:n_dim) - x_prev) + G2*(y_prev - ref);
        x_prev = x_est(1:n_dim);
    else
        % Use real state (assumed known, perfect information)
        u = u_prev + G1*(x - x_prev) + G2*(y_prev - ref);
        x_prev = x;
    end

    % Store the values of state, measurements and input as old values for
    % the next iteration of control calculations.
    y_prev = y_meas;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate thruster distribution %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p = [u; fmin; fmax; beta];
    
    z = quadprog(Phi_quad, R_quad*p, A2, C2*p, A1, C1*p, [], [], [], options);
    
    % Get control forces
    f = z(1:n_thrusters);

    % Store control input and individual thruster forces
    rpm_array(:,i) = inv(K_force)*f;
    f_array(:,i) = f;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate external disturbances %%% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    %%%%%%%%%%%%%%%%%%%%
    %%% Update model %%%
    %%%%%%%%%%%%%%%%%%%%
    switch (integration_method)
        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            xdot = supply_model(t, x, u, M, D, wind_force_array(:,i), wave_force_array(:,i), current_force_array(:,i));
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind_force_array(:,i), wave_force_array(:,i), current_force_array(:,i)), t, x, dt);
    end

    % Measurement with added noise
    % This is the measurement for timestep k+1
    if (use_noise_in_measurements)
        y_meas = x(1:3) + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = x(1:3);
    end

    % Update measurement
    y_meas_array(:,i+1) = y_meas;

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

    else
        % Use state process value
        x_est(1:n_dim) = x;
    end

    % Update time
    t = t + dt;

    % Update previous control signal
    u_prev = u;
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    x_est_array(:,i+1) = x_est;
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

    % Environmental force in BODY coordinate system
    env_dist = wind_force_array(:,i) + wave_force_array(:,i) + current_force_array(:,i);

    % Update thruster plots
    animate_combined.UpdatePlot(t_array(i), x_array(3,i), f, thruster_angles, 1e4, ...
        u(1), u(2), u(3), env_dist(1), env_dist(2), env_dist(3), 1e4, 1e4, 1e5);

end

% Plot data
if (n_thrusters == 3)
    plot_supply_lq_alloc_1tunnel(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, ...
        current_force, wave_force, rpm_array, f_array, setpoint, true, folder, file_prefix);
elseif (n_thrusters == 4)
    plot_supply_lq_alloc_2tunnel(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, ...
        current_force, wave_force, rpm_array, f_array, setpoint, true, folder, file_prefix);
end

% Store workspace
if (store_workspace)
    % Create folder if it does not exists
    if (not(isfolder("Workspace")))
        mkdir("Workspace");
    end
    save("Workspace/" + workspace_file_name);
end
