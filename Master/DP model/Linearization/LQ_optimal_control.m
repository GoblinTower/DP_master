% Script for testing LQ optimal control for a linearized ssm 
% using Taylor series expansion.
clear, clc, close all;

addpath("..\Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_linearized';

% Preallocate arrays
t = 0;
t_array = zeros(1,N+1);   % Time
u_array = zeros(3,N);     % Control input
u = zeros(3,1);

% Real process
x_array = zeros(6,N+1);   % States 
x_array(:,1) = x0;
x = x0;

% Initiale values
x_prev = zeros(6,1);
u_prev = zeros(3,1);
y_prev = zeros(3,1);

y_meas = x0(1:3);

% Store gain
K_array = zeros(9*3,N);

% Get linearized matrices
[A_lin, B_lin, C_lin] = linearizing_model(x_op, u_op);

% Compute LQ optimal control
% Need to write matrices on deviation form
% Refer to the paper "Discrete LQ optimal control with integral action"
% by David de Ruscio
n_dim = size(A_lin, 1);
m_dim = size(C_lin, 1);
r_dim = size(B_lin, 2);

A_dev = [A_lin, zeros(n_dim, m_dim); C_lin, eye(m_dim, m_dim)];
B_dev = [B_lin; zeros(m_dim, r_dim)];
C_dev = [C_lin, eye(m_dim, m_dim)];

Q_dev = C_dev'*Q*C_dev;

% Calculate gain matrix
[K,~,~] = dlqr(A_dev, B_dev, Q_dev, R);
G1 = -K(:,1:n_dim);
G2 = -K(:,n_dim+1:n_dim+m_dim);

for i=1:N
    
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
            [~, x] = runge_kutta_4(@(t, x) supply(x, u), t, x, dt);
    end

    % Measurement with added noise
    if (use_noise_in_measurements)
        y_meas = x(1:3) + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = x(1:3);
    end

    % Control input using LQ
    ref = setpoint(:,i);
    u = u_prev + G1*(x - x_prev) + G2*(y_prev - ref);

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    if (run_kalman_filter)
        K = X_apriori*C_dev'*inv(C_dev*X_apriori*C_dev' + W);
        X_posteriori = (eye(9) - K*C_dev)*X_apriori*(eye(9) - K*C_dev)' + K*W*K';
        X_apriori = A_dev*X_posteriori*A_dev' + V;
    
        delta_x = delta_x + K*(y_meas - y_lin_hat);

        % Store data
        K_array(:,i) = K(:);
    end

    % Update time
    t = t + dt; 
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    u_array(:,i) = u;

    x_prev = x;
    u_prev = u;
    y_prev = y_meas;
  
end

% Plot data
plot_supply_lq_linearized(t_array, x_array, K_array, u_array, setpoint, true);
