% Nonlinear MPC applied to supply model
% Two models can be used in the model based comparison in MPC
% DP model by Fossen and reduced DP model (without the error term, b)
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_non_linear_mpc_control';

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

% Initial values
x = x0;                         % Initial real state 
x_est = x0_est;                 % Initial state estimate
y_meas = y0_meas;               % Initial measured value         

t = 0;                          % Current time

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros(6*3,N);         % Storing Kalman filter gain

for i=1:N

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate control signal u %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%E1
    ref = setpoint(:,i:(i+horizon_length-1));

    % Solve non-linear optimization problem
    if (run_kalman_filter)
        % Use state from Kalman filter
        u_sol = fmincon(@(u) non_linear_objective_function(u, ref, M, D, P, Q, x_est, horizon_length, dt, 1), u0, [], [], [], [], [], [], [], options);
    else
        % Use real state (assumed known, perfect information)
        % b is unknown, and hence set to zero
        x_extended = [x; 0; 0; 0];
        u_sol = fmincon(@(u) non_linear_objective_function(u, ref, M, D, P, Q, x_extended, horizon_length, dt, 1), u0, [], [], [], [], [], [], [], options);
    end

    % ADD SOME TEST CODE HERE
    % FOR INTEGRAL GAIN

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
    if (run_kalman_filter)

        [Ad, Bd, Cd, Ad_dot, Bd_dot, Cd_dot] = dp_model_discrete_matrices(x_est, u, dt, M, D);
        
        % Measurement model update (time, k)
        y_apriori = x_est(1:3);
        
        % Calculate Kalman gain
        K = p_apriori*Cd_dot'*inv(Cd_dot*p_apriori*Cd_dot' + W);

        % Aposteraiori state estimate (time, k)
        x_aposteriori = x_apriori + K*(y_meas_array(:,i) - y_apriori);

        % Apropri state estimate (time, k+1)
        x_apriori = Ad*x_est + Bd*u;

        % Aposteriori state error covariance matrix (time, k)
        p_aposteriori = (eye(9) - K*Cd_dot)*p_apriori*(eye(9) - K*Cd_dot)' + K*W*K';
        
        % Apriori state error covariance matrix (time, k+1)
        p_apriori = Ad_dot*p_aposteriori*Ad_dot' + V;

        x_est = x_apriori;
    end

    % Update time
    t = t + dt;
    disp(['Current time: ', num2str(t)]);
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    x_est_array(:,i+1) = x_est;
    y_meas_array(:,i+1) = y_meas;
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
plot_supply_nmpc_no_disturbance(t_array, x_array, K_array, u_array, setpoint, true);
