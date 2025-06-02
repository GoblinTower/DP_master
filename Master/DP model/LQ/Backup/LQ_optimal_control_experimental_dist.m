% Script implementing LQ optimal control for the supply model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_control_experimental_dist';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1,N+1);                 % Time array

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

% Create linear kalman filter
kalman = LinearKalmanFilter(x_aposteriori, p_aposteriori);

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros(n_kal_dim*3,N);   % Storing Kalman filter gain


%%% DELETE LATER
dx = zeros(6,1);
w = 0;

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
    wind = wind_force_calc(wind_abs(i), wind_beta(i), x(3), x(4), x(5), rho, Af, Al, L, Cx, Cy, Cn);

    %%%%%%%%%%%%%%%%%%%%
    %%% Update model %%%
    %%%%%%%%%%%%%%%%%%%%
    switch (integration_method)
        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            xdot = supply(x,u);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind, wave_force(:,i), current_force(:,i)), t, x, dt);
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
    if true % (run_kalman_filter)
        
        % First timestep
        if (i==1)
            dy = zeros(3,1);
            du = zeros(3,1);
        else
            dy = y_meas_array(:,i+1) - y_meas_array(:,i);
            du = u - u_array(:,i-1);
        end

        % Get Kalman gain from inbuild MATLAB function dlqe
        % [K,~,~,~] = dlqe(A_lin, eye(6), C_lin, W, V); 

        % Update filter
        % [dx_est, Pcov, K] = kalman.UpdateFilter(du, dy, A_lin, B_lin, C_lin, W, V);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% My (attempt at) Kalman filter %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Calculate state apriori vector (k+1)
        % x_apriori = A_lin*x_aposteriori + B_lin*du;
        
        % Calculate state aposteriori vector (k+1)
        % x_aposteriori = x_apriori + K*(dy - C_lin*x_apriori);
        
        % x_aposteriori = A_lin*x_aposteriori + B_lin*du + K*(dy - C_lin*x_aposteriori);
        % dx_est = x_aposteriori;

        % Calculate estimate
        % x_est = dx + x_est;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Direct copy from paper equation (uses delta_y(k)) %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % dx = A_lin*dx + B_lin*du + K*(dy - C_lin*dx);
        % x_est = dx + x_est;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Normal Kalman filter, assumes white noise %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % x_est = A_lin*x_est + B_lin*u + K*(y_meas - C_lin*x_est);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Kalman filter (estimating noise) %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % if (i > 1)
        %     dw = x_est - A_lin*x_prev_kal - B_lin*u - w;
        %     w = w + dw;
        % end
        % % Store x_est before updating
        % x_prev_kal = x_est;
        % 
        % % Calculate x apriori
        % xap = A_lin*x_est + B_lin*u + w;
        % 
        % % Estimate x aposteriori
        % x_est = xap + K*(y_meas - C_lin*xap);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Augmented SSM to include noise integration %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % [At, Bt, Gt, Ct] = include_integrator_ssm(A_lin, B_lin, C_lin);
        % 
        % [K,~,~,~] = dlqe(At, Gt, Ct, W, V);
        % 
        % % Calculate x apriori
        % xap = At*x_est + Bt*u;
        % 
        % % Estimate x aposteriori
        % x_est = xap + K*(y_meas - Ct*xap);
        % 
        % % Store data
        % K_array(:,i) = K(:);

        % n_dim = size(A_lin,1);
        % r_dim = size(B_lin,2);
        % m_dim = size(C_lin,1);
        
        % At = [A_lin, eye(n_dim); zeros(n_dim), eye(n_dim)];      % Augmented state transition matrix
        % Bt = [B_lin; zeros(n_dim,r_dim)];                        % Augmented input matrix
        % Gt = [zeros(n_dim); eye(n_dim)];                         % Augmented process noise matrix
        % Ct = [C_lin, zeros(m_dim, n_dim)];                       % Augmented output matrix

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Augmented SSM to include noise integration modified %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         G_lin = [zeros(3); eye(3)];
        [At, Bt, Gt, Ct] = include_integrator_ssm_with_specified_noise(A_lin, B_lin, G_lin, C_lin);
        [K,~,~,~] = dlqe(At, Gt, Ct, eye(3), eye(3));

        % Calculate x apriori
        xap = At*x_est + Bt*u;
  
        % Estimate x aposteriori
        x_est = xap + K*(y_meas - Ct*xap);

        % Store data
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
plot_supply_lq_no_disturbance(t_array, x_array, K_array, u_array, setpoint, true);
