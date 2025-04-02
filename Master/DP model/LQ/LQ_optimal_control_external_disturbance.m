% Script for implementing and testing Kalman filter
clear, clc, close all;

addpath("Plots\");
addpath("..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_control_ext_disturbances';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Create continuous time-varying DP model for DP
Bc = [zeros(3,3); inv(M)];
Cc = [eye(3), zeros(3,3)];

% Preallocate arrays
t = 0;
t_array = zeros(1,N+1);   % Time
u_array = zeros(3,N);     % Control input
u = zeros(3,1);
wind_force= zeros(3,1);

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

for i=1:N

    Rotation = rotation_matrix(y_meas(3));

    Ac = [
        zeros(3,3), Rotation; 
        zeros(3,3), -inv(M)*D; 
    ];

    % Create discrete matrices
    sys = ss(Ac, Bc, Cc, 0);
    sysd = c2d(sys, dt);
    
    A_lin = sysd.A;
    B_lin = sysd.B;
    C_lin = sysd.C;

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
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind, current_force(:,i)), t, x, dt);
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
    wind_force(:,i) = wind;

    x_prev = x;
    u_prev = u;
    y_prev = y_meas;
  
end

% Plot data
plot_supply_lq_disturbance(t_array, x_array, K_array, u_array, wind_abs, wind_beta, wind_force, current_force, setpoint, true);
