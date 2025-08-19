% Script implementing LQ optimal control for the supply model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_control_without_disturbance';
% run 'Scenarios\supply_scenario_LQ_control_with_disturbance';

% Type of sys_identification
sysid = 'dsr';
% sysid = 'dsr_e';
% sysid = 'pem';

run_continous_version = true;

% Save file names and location
folder = 'Results/LQ_supply';
file_prefix = strcat(sysid, simulation_type);

% Create model using DSR generated matrices
if (run_continous_version)
    load_path = strcat('Log\', sysid, '_cont_ssm_supply');
else
    load_path = strcat('Log\', sysid, '_ssm_supply');
end
si = load(load_path);
A_si = si.A;
B_si = si.B;
C_si = si.C;

dt = si.dt;

if (run_continous_version)
    sysd = ss(A_si, B_si, C_si, 0, dt);
    sys = d2c(sysd);
    A_si = sys.A;
    B_si = sys.B;
    C_si = sys.C;
end

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by Thor
% Inge Fossen et al (1995).
[~, ~, M, D] = supply();

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

u_array = zeros(3,N);                   % Control input array

u_prev = zeros(3,1);                    % Previous control signal

% Initial values
x_prev = zeros(n_dim_control,1);        % Previus state 
y_prev = zeros(3,1);                    % previous output
x = x0;                                 % Initial real state 
x_est = x0_est;                         % Initial state estimate
y_meas = y0_meas;                       % Initial measured value         

t = 0;                                  % Current time

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros(n_kal_dim*3,N);   % Storing Kalman filter gain

for i=1:N

    % Get vessel heading
    psi = y_meas(3);
    
    % Calculate discrete supply model matrices
    if (run_continous_version)
        [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_from_cont_si(A_si, B_si, C_si, psi, dt);
    else
        [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si(A_si, B_si, C_si, psi, dt);
    end 

    % Calculate LQ gain on deviation form
    [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P);

    % Control input using LQ
    ref = setpoint(:,i);
    if (run_kalman_filter)
        % Use state from Kalman filter
        u = u_prev + G1*(x_est(1:n_dim_control) - x_prev) + G2*(y_prev - ref);
        x_prev = x_est(1:n_dim_control);
    else
        error('Must run Kalman filter');
    end

    % Store the values of state, measurements and input as old values for
    % the next iteration of control calculations.
    y_prev = y_meas;

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
        if (run_continous_version)
            [A_lin, B_lin, F_lin, C_lin] = dp_model_discrete_matrices_from_cont_si_int(A_si, B_si, C_si, psi, dt);
        else
            [A_lin, B_lin, F_lin, C_lin] = dp_model_discrete_matrices_si_int(A_si, B_si, C_si, psi, dt);
        end
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
        error('Must run Kalman filter');
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
    disp(['Integrator term : ', 'b(1): ', num2str(x_est(10)), ' b(2): ', num2str(x_est(11)), ...
        ' b(3): ', num2str(x_est(12))]);

    % Update animated positon plot
    if (animate_kalman_estimate)
        animate_kalman.UpdatePlot(t_array(i), x_est_array(1,i), x_est_array(2,i), x_est_array(3,i),...
            y_meas_array(1,i), y_meas_array(2,i), y_meas_array(3,i),...
            setpoint(1,i), setpoint(2,i), setpoint(3,i));
        
        pause(animation_delay);
    end

end

% Plot data
plot_dp_model_lq(t_array, x_array, x_est_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, current_force, wave_force, setpoint, true, folder, file_prefix);

% Store data
save_destination = strcat(folder, '/', file_prefix, '_data');
save(save_destination);