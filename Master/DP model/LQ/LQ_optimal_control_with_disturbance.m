% Script implementing LQ optimal control for the supply model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_control_with_disturbance';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by Thor
% Inge Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1,N+1);                 % Time array
 
wind_force_array = zeros(3,N);          % Wind force array

x_array = zeros(n_dim,N+1);             % State array from real process (unknown)
x_array(:,1) = x0;                      % Storing initial value of state

x_est_array = zeros(n_kal_dim,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;              % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);            % Measurement array
y_meas_array(:,1) = y0_meas;            % Storing initial value of measurement 

u_array = zeros(3,N);                   % Control input array

% Initial values
x_prev = zeros(n_dim,1);                % Previus state 
y_prev = zeros(3,1);                    % previous output
u_prev = zeros(3,1);                    % Previous control signal

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

    % Get wind forces and momentum
    % The actual wind forces will depend on the real ship position, hence
    % we use the state variables from the 'real' process.
    wind_force = wind_force_calc(wind_abs(i), wind_beta(i), x(3), x(4), x(5), rho, Af, Al, L, Cx, Cy, Cn);
    wind_force_array(:,i) = wind_force;

    %%%%%%%%%%%%%%%%%%%%
    %%% Update model %%%
    %%%%%%%%%%%%%%%%%%%%
    switch (integration_method)
        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            xdot = supply_model(t, x, u, M, D, wind_force, wave_force(:,i), current_force(:,i));
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind_force, wave_force(:,i), current_force(:,i)), t, x, dt);
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

        if (kalman_model == KalmanModel.DeviationForm)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Kalman filter on deviation form %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Need deviation in y and u
            if (i==1)
                dy = zeros(3,1);
                du = zeros(3,1);
            else
                dy = y_meas_array(:,i+1) - y_meas_array(:,i);
                du = u - u_array(:,i-1);
            end

            % Get Kalman gain from inbuilt MATLAB function dlqe
            [K,~,~,~] = dlqe(A_lin, G_lin, C_lin, W, V);

            % Calculate dx_apriori at time step (i+1)
            dx_apriori = A_lin*dx_aposteriori + B_lin*du;
  
            % Calculate dx_aposteriori at time step (i+1)
            dx_aposteriori = dx_apriori + K*(dy - C_lin*dx_apriori);

            % Calculate x_est at time step (i+1)
            x_est = dx_aposteriori + x_est;

        elseif (kalman_model == KalmanModel.IntegratorIncluded)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Augmented SSM to include noise integration %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Get Kalman gain from inbuild MATLAB function dlqe
            [At, Bt, Gt, Ct] = include_integrator_ssm_with_specified_noise(A_lin, B_lin, G_lin, C_lin);
            [K,~,~,~] = dlqe(At, Gt, Ct, eye(3), eye(3));

            % Calculate x_apriori at time step (i+1)
            x_apriori = At*x_aposteriori + Bt*u;
  
            % Calculate x_aposteriori at time step (i+1)
            x_aposteriori = x_apriori + K*(y_meas - Ct*x_apriori);

            x_est = x_aposteriori;

        elseif (kalman_model == KalmanModel.Normal)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% SSM model assuming white process noise term %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
            % Get Kalman gain from inbuilt MATLAB function dlqe
            [K,~,~,~] = dlqe(A_lin, G_lin, C_lin, W, V);

            % Calculate dx_apriori at time step (i+1)
            x_apriori = A_lin*x_aposteriori + B_lin*u;
  
            % Calculate dx_aposteriori at time step (i+1)
            x_aposteriori = x_apriori + K*(y_meas - C_lin*x_apriori);

            x_est = x_aposteriori;

        end

        % Store data Kalman gain
        K_array(:,i) = K(:);

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

    % Update animated positon plot
    if (animate_kalman_estimate)
        animate_kalman.UpdatePlot(t_array(i), x_est_array(1,i), x_est_array(2,i), x_est_array(3,i),...
            y_meas_array(1,i), y_meas_array(2,i), y_meas_array(3,i),...
            setpoint(1,i), setpoint(2,i), setpoint(3,i));
        
        pause(animation_delay);
    end

end

% Plot data
plot_supply_lq_disturbance(t_array, x_array, K_array, u_array, wind_abs, wind_beta, wind_force_array, current_force, wave_force, setpoint, true);
