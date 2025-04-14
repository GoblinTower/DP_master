% Script implementing LQ optimal control for the OSV model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\osv_scenario_LQ_control';

% Create model using DSR generated matrices
dsr = load('Log\ssm_dsr_osv.mat');
A_si = dsr.A;
B_si = dsr.B;
C_si = dsr.C;

% Preallocate arrays
t_array = zeros(1,N+1);              % Time array

x_array = zeros(12,N+1);             % State array from real process (unknown)
x_array(:,1) = x0;                   % Storing initial value of state

x_est_array = zeros(n_dim+6,N+1);    % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;           % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);         % Measurement array
y_meas_array(:,1) = y0_meas;         % Storing initial value of measurement 

u_array = zeros(3,N);                % Control input array

% Initial values
x_prev = zeros(n_dim+6,1);           % Previus state 
y_prev = zeros(3,1);                 % previous output
u_prev = zeros(3,1);                 % Previous control signal
    
x = x0;                              % Initial real state 
x_est = x0_est;                      % Initial state estimate
y_meas = y0_meas;                    % Initial measured value         

t = 0;                               % Current time

% Create linear kalman filter
kalman = LinearKalmanFilter(x_aposteriori, p_aposteriori);

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Store Kalman gain
K_array = zeros((n_dim+6)*3,N);   % Storing Kalman filter gain

for i=1:N

    % Calculate vessel heading
    psi = y_meas(3);

    % Calculate discrete DP model matrices
    [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si(A_si, B_si, C_si, psi, dt);

    % Calculate LQ gain on deviation form
    [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P);

    % Control input using LQ
    ref = setpoint(:,i);

    % Use state from Kalman filter
    u = u_prev + G1*(x_est - x_prev) + G2*(y_prev - ref);
    x_prev = x_est;

    % Calculate RPM input to OSV model
    u_mod = [sign(u(1))*sqrt(abs(u(1))); 0; sign(u(2))*sqrt(abs(u(2)));...
        sign(u(3))*sqrt(abs(u(3))); deg2rad(azimuth_angle_1); deg2rad(azimuth_angle_2)];

    % Store the values of state, measurements and input as old values for
    % the next iteration of control calculations.
    y_prev = y_meas;

    %%%%%%%%%%%%%%%%%%%%
    %%% Update model %%%
    %%%%%%%%%%%%%%%%%%%%
    switch (integration_method)
        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            xdot = osv(x, u_mod);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) osv(x, u_mod), t, x, dt);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Delta u
    du = u - u_prev;
    
    % Delta y
    if (i==1)
        dy = zeros(3,1);
    else
        dy = y_meas_array(:,i) - y_meas_array(:,i-1);
    end

    % Update filter
    [dx_est, Pcov, K] = kalman.UpdateFilter(du, dy, A_lin, B_lin, C_lin, W, V);

    % Store data
    K_array(:,i) = K(:);

    % Calculate estimate
    x_est = dx_est + x_est;

    % Measurement with added noise
    if (use_noise_in_measurements)
        y_meas = [x(7);x(8);x(12)] + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = [x(7);x(8);x(12)];
    end

    % Update time
    t = t + dt;

    % Update previous control signal
    u_prev = u;
    
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
plot_supply_lq_no_disturbance(t_array, x_array, K_array, u_array, setpoint, true);
