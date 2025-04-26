% Script implementing LQ optimal control for the supply model
clear, clc, close all;

addpath("Plots\");
addpath("..\..\Tools\");

% Load configuration data
run 'Scenarios\supply_scenario_LQ_control_azimuth';

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% Preallocate arrays
t_array = zeros(1,N+1);         % Time array

x_array = zeros(6,N+1);         % State array from real process (unknown)
x_array(:,1) = x0;              % Storing initial value of state

x_est_array = zeros(6,N+1);     % Storing estimated states (from Kalman, known)
x_est_array(:,1) = x0_est;      % Storing initial assumed value of state

y_meas_array = zeros(3,N+1);    % Measurement array
y_meas_array(:,1) = y0_meas;    % Storing initial value of measurement 

u_array = zeros(3,N);           % Control input array

% Initial values
x_prev = zeros(6,1);      % Previus state 
y_prev = zeros(3,1);      % previous output
u_prev = zeros(3,1);      % Previous control signal

x = x0;                   % Initial real state 
x_est = x0_est;           % Initial state estimate
y_meas = y0_meas;         % Initial measured value         

t = 0;                    % Current time

% Create linear kalman filter
kalman = LinearKalmanFilter(x_aposteriori, p_aposteriori);

% Create Kalman animation
if (animate_kalman_estimate)
    animate_kalman = AnimateKalman();
end

% Thruster allocation
rpm_array = zeros(3,N);              % RPM array
angle_array = zeros(1,N);            % Azimuth angle

alpha0 = deg2rad(0);                 % Azimuth starts in zero position

% Initial guess of thruster optimization variable
z0 = zeros(7,1);

% Store Kalman gain
K_array = zeros(6*3,N);   % Storing Kalman filter gain

for i=1:N

    % Calculate vessel heading
    psi = y_meas(3);

    % Calculate discrete supply model matrices
    [A_lin, B_lin, C_lin] = supply_discrete_matrices(M, D, psi, dt, false);

    % Calculate LQ gain on deviation form
    [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P);

    % Control input using LQ
    if (run_kalman_filter)
        % Use state from Kalman filter
        ref = setpoint(:,i);
        u = u_prev + G1*(x_est - x_prev) + G2*(y_prev - ref);
        x_prev = x_est;
    else
        % Use real state (assumed known, perfect information)
        ref = setpoint(:,i);
        u = u_prev + G1*(x - x_prev) + G2*(y_prev - ref);
        x_prev = x;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Calculate thruster distribution %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Non-linear constraint function
    nonlin = @(z_) non_linear_constraints_alloc(z_, u, alpha0, fmin, fmax, ...
        alpha_min, alpha_max, alpha_diff_min, alpha_diff_max);

    options = optimoptions('fmincon','Algorithm','sqp');
    % Optimization
    z_sol = fmincon(@(z) cost_function_thruster_alloc(z, alpha0, P_thr, W_thr, ...
        Q_thr, Omega_thr, rho, epsilon), z0, [], [], [], [], [], [], nonlin, options); %, options);
    
    % Thruster forces
    f = z_sol(1:3);

    % Update previous azimuth position
    alpha0 = z_sol(4);

    % Store solution for warm startup
    z0 = z_sol;

    % Store angle
    angle_array(:,i) = alpha0;
    
    if (linear_force_rpm_relation)
        % Linear relation
        rpm_array(:,i) = inv(K_force)*f;
    else
        % Quadratic relation
        quad = inv(K_force)*f;
        rpm_array(:,i) = sign(quad)*sqrt(abs(quad));
    end

    % Store the values of state, measurements and input as old values for
    % the next iteration of control calculations.
    y_prev = y_meas;

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

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Update Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    if (run_kalman_filter)
        
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
       
    end

    % Measurement with added noise
    if (use_noise_in_measurements)
        y_meas = x(1:3) + normrnd(measurement_noise_mean, measurement_noise_std, 3, 1);
    else
        y_meas = x(1:3);
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
plot_supply_lq_alloc_azimuth_no_disturbance(t_array, x_array, K_array, u_array, rpm_array, angle_array, setpoint, true);
