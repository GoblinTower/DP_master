% Script for implementing and testing non-linear MPC
clear, clc, close all;

addpath("Plots\");
addpath("..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_non_linear_mpc_control_ext_disturbances';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Create continuous time-varying DP model for DP
Bc = [zeros(3,3); inv(M); zeros(3,3)];
Cc = [eye(3), zeros(3,3), zeros(3,3)];

% Dimension of semi-linear model
r_dim = size(Bc, 2);
m_dim = size(Cc, 1);
n_dim = size(Bc, 1);

u0 = zeros(r_dim, horizon_length);

% Preallocate arrays
t = 0;
t_array = zeros(1, N+1);       % Time
u_array = zeros(r_dim, N);     % Control input

% Real process
x_array = zeros(6,N+1);   % States 
x_array(:,1) = x0;
x = x0;

% Linear model (See chapter 6.7.3 in Handbook of Marine Craft
% Hydrodynamics and Motion Control, second edition, by Thor I. Fossen)
x_lin_array = zeros(n_dim, N+1);
x_lin_array(:,1) = x_lin0;
x_lin = x_lin0;

y_meas = x0(1:3);

% Store gain
K_array = zeros(n_dim*m_dim, N);

for i=1:N

    R = rotation_matrix(y_meas(3));
    
    Ac = [
        zeros(3,3), R, zeros(3,3); 
        zeros(3,3), -inv(M)*D, -inv(M)*R; 
        zeros(3,3), zeros(3,3), zeros(3,3)
    ];

    % Create discrete matrices
    sys = ss(Ac, Bc, Cc, 0);
    sysd = c2d(sys, dt);
    
    A_lin = sysd.A;
    B_lin = sysd.B;
    C_lin = sysd.C;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate control signal u %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%E1
    ref = setpoint(:,i:(i+horizon_length-1));

    % Solve non-linear optimization problem
    u_sol = fmincon(@(u) non_linear_objective_function(u, ref, M, D, P, Q, x_lin, horizon_length, dt, 1), u0, [], [], [], [], [], [], [], options);
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
            [~, x] = runge_kutta_4(@(t, x) supply_model(t, x, u, M, D, wind, current_force(:,i)), t, x, dt);
    end

    % Measurement with added noise
    if (use_noise_in_measurements)
        y_meas = x(1:3) + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = x(1:3);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update linear model %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_lin = A_lin*x_lin + B_lin*u;
    y_lin = C_lin*x_lin; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    if (run_kalman_filter)
        K = X_apriori*C_lin'*inv(C_lin*X_apriori*C_lin' + W);
        X_posteriori = (eye(n_dim) - K*C_lin)*X_apriori*(eye(n_dim) - K*C_lin)' + K*W*K';
        X_apriori = A_lin*X_posteriori*A_lin' + V;
    
        x_lin = x_lin + K*(y_meas - y_lin);

        % Store data
        K_array(:,i) = K(:);
    end

    % Update time
    t = t + dt
    
    % Store data
    t_array(i+1) = t;
    u_array(:,i) = u;
    x_array(:,i+1) = x;
    x_lin_array(:,i+1) = x_lin;
    
end

% Plot data
plot_supply_kalman(t_array, x_array, x_lin_array, K_array, u_array);
