% Script implementing LQ optimal control for the supply model

addpath("Plots\");
addpath("..\..\Tools\");
addpath("Scenarios\Azimuth\")

if (exist('external_scenario', 'var'))
   run 'Scenarios\Azimuth\supply_scenario_LQ_control_azimuth';
else
    clear, clc, close all;
    % Load configuration data
    run 'Scenarios\Azimuth\supply_scenario_LQ_control_azimuth';
end

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

if (~exist('external_scenario', 'var'))
    % Create Kalman animation
    if (animate_kalman_estimate)
        animate_kalman = AnimateKalman();
    end

    % Thruster allocation
    % animate_forces = AnimateForces(70, 8);
    % animate_thrusters = AnimateThrusters(70, 8, thruster_positions, thruster_names);
    animate_combined = AnimateCombined(70, 8, thruster_positions, thruster_names);
end

rpm_array = zeros(n_thrusters,N);                 % RPM array
thruster_angles = zeros(n_thrusters, N);          % Thruster angles
f_array = zeros(n_thrusters,N);                   % Force from thrusters

angle_array = zeros(n_azimuths,N);                % Azimuth angles
slack_array = zeros(3,N);                         % Slack variables

% Initial guess of thruster optimization variable

z0 = zeros(z_dim,1);

% Store Kalman gain
K_array = zeros(n_kal_dim*3,N);   % Storing Kalman filter gain

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
    % Non-linear constraint function
    nonlin = @(z_) non_linear_constraints_alloc(z_, u, alpha0, fmin, fmax, ...
        alpha_min, alpha_max, alpha_diff_min, alpha_diff_max, @(a) thruster_configuration_matrix(a));

    % Optimization
    z_sol = fmincon(@(z) cost_function_thruster_alloc(z, alpha0, P_thr, W_thr, ...
        Q_thr, Omega_thr, rho, epsilon, @(a) thruster_configuration_matrix(a)), z0, [], [], [], [], [], [], nonlin, options); %, options);
    
    % Thruster forces
    f = z_sol(1:n_thrusters);

    % Update previous azimuth position
    alpha0 = z_sol(n_thrusters+1:n_thrusters+n_azimuths);

    % Store solution for warm startup
    z0 = z_sol;

    % Store angles and thruster force, control input to thrusters and slack
    % variables
    angle_array(:,i) = alpha0;
    thruster_angles = [0; 0; alpha0(1); alpha0(2)];
    f_array(:,i) = f;
    rpm_array(:,i) = inv(K_force)*f;
    slack_array(:,i) = z_sol(7:9);

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

    if (~exist('external_scenario', 'var'))
        % Output data
        disp(['Current time: ', num2str(t)]);
        disp(['Integrator term : ', 'b(1): ', num2str(x_est(7)), ' b(2): ', num2str(x_est(8)), ...
            ' b(3): ', num2str(x_est(9))]);
    
        % Update animated positon plot
        if (animate_kalman_estimate)
            animate_kalman.UpdatePlot(t_array(i), x_est_array(1,i), x_est_array(2,i), x_est_array(3,i),...
                y_meas_array(1,i), y_meas_array(2,i), y_meas_array(3,i),...
                setpoint(1,i), setpoint(2,i), setpoint(3,i));
        end

        % Environmental force in BODY coordinate system
        env_dist = wind_force_array(:,i) + wave_force_array(:,i) + current_force_array(:,i);
    
        % animate_forces.UpdatePlot(x_array(3,i), u(1), u(2), u(3), 1e4, 1e4, 1e5);
        % animate_thrusters.UpdatePlot(t_array(i), x_array(3,i), f, [0, 0, alpha0(1), alpha0(2)], 1e4);
        animate_combined.UpdatePlot(t_array(i), x_array(3,i), f, [0, 0, alpha0(1), alpha0(2)], 1e4, ...
            u(1), u(2), u(3), env_dist(1), env_dist(2), env_dist(3), 1e4, 1e4, 1e5);

        pause(animation_delay);
    end
end

if (~exist('external_scenario', 'var'))
    % Plot data
    plot_supply_lq_alloc_azimuth(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, ...
            current_force, wave_force, rpm_array, f_array, angle_array, slack_array, setpoint, true, folder, file_prefix);
end

% Store workspace
if (store_workspace)
    % Create folder if it does not exists
    if (not(isfolder("Workspace")))
        mkdir("Workspace");
    end
    save("Workspace/" + workspace_file_name);
end
