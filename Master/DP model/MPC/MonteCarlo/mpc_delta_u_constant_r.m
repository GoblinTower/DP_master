% Script for implementing and testing model predictive controller (MPC)
% Runs linear MPC with changing state transition matrix. It is assumed
% that the yaw velocity is constant for the entire horizon. Thus the yaw
% when calculating any matrix A(t) is psi(t) = psi(t0) + dpsi/dt(t0)*(t-t0)
addpath("..\Plots\");
addpath("..\..\..\Tools\");

if (exist('external_scenario', 'var'))
    run 'Scenarios\supply_scenario_mpc_r_const_with_disturbance';
else
    clear, clc, close all;
    % Load configuration data
    run '..\Scenarios\du_formulation\supply_scenario_mpc_r_const_without_disturbance';
    % run '..\Scenarios\du_formulation\supply_scenario_mpc_r_const_with_disturbance';
end

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Initial guess for the MPC optimization problem
z0 = zeros(horizon_length*(2*r_dim + n_kal_dim + 2*m_dim),1);

% Preallocate arrays
t_array = zeros(1,N+1);                 % Time array

wind_force_array = zeros(3,N);          % Wind force array relative to BODY coordinate frame
wave_force_array = zeros(3,N);          % Wave force array relative to BODY coordinate frame
current_force_array = zeros(3,N);       % Current force array relative to BODY coordinate frame

x_array = zeros(n_dim,N+1);             % State array from real process (unknown)
x_array(:,1) = x0;                      % Storing initial value of state

x_est_array = zeros(n_kal_dim,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;              % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);            % Measurement array
y_meas_array(:,1) = y0_meas;            % Storing initial value of measurement 

u_array = zeros(r_dim,N);               % Control input array

% Initial values
x = x0;                                 % Initial real state 
x_est = x0_est;                         % Initial state estimate
y_meas = y0_meas;                       % Initial measured value         

t = 0;                                  % Current time

if (~exist('external_scenario', 'var'))
    % Create Kalman animation
    if (animate_kalman_estimate)
        animate_kalman = AnimateKalman();
    end
end

% Store Kalman gain
K_array = zeros(n_kal_dim*3,N);         % Storing Kalman filter gain

for i=1:N

    % Get vessel heading
    psi = y_meas(3);

    % Calculate discrete dp model matrices
    [A_lin, B_lin, F_lin, C_lin] = dp_fossen_discrete_matrices(M, D, psi, dt, false);

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% External forces %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    % Get wind forces and momentum
    % The actual wind forces will depend on the real ship position, hence
    % the state variables from the 'real' process is used.
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
    ref = ref(:);                               % Must be a column vector
             
    % Force and momentum limitations
    if (use_force_limitation)
        if (i==1)
            u_minus_1 = u_array(:,i);
        else
            u_minus_1 = u_array(:,i-1);
        end
        [Ai, bi] = calculate_force_inequality_matrix(horizon_length, u_minus_1, n_kal_dim, m_dim, r_dim, max_inputs, max_delta_u);
    else
        Ai = []; 
        bi = [];
    end

    % Solve quadratic optimization problem
    [H, c, Ae, be] = calculate_mpc_delta_u_form_constant_rotation_rate_dist(P, Q, A_lin, B_lin, C_lin, F_lin, tau, x_est, u_prev, horizon_length, ref, dt, M, D);

    % Optimization solver
    z = quadprog(H, c, Ai, bi, Ae, be, [], [], z0, options);
    
    % Store previous value for warm starting
    z0 = z; 

    % Get control signal
    u = z(1:r_dim);

    % Store control signal for future use
    u_prev = u;

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
        x_est = [x; 0; 0; 0];
    end

    % Update time
    t = t + dt;
    
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
            
            pause(animation_delay);
        end
    end
end

if (~exist('external_scenario', 'var'))
    % Plot data
    plot_supply_linear_mpc_r_constant(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, current_force, wave_force, setpoint, true, folder, file_prefix);
end

% Store workspace
if (store_workspace)
    % Create folder if it does not exists
    if (not(isfolder("Workspace")))
        mkdir("Workspace");
    end
    save(strcat("Workspace/", workspace_file_name, '_', num2str(mc_iteration)), "x_array", "t_array", "u_array", "setpoint", "K_array", "wind_abs", "wind_beta", "wind_force_array", ...
        "current_force", "wave_force", "x_est_array");
end