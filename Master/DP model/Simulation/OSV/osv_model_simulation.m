clear, clc, close all;

addpath("Plots\");
addpath("..\..\..\Tools\");

% Load configuration data
% run 'Scenarios\osv_constant_input';
% run 'Scenarios\osv_variable_input_prbs';
% run 'Scenarios\osv_variable_input_prbs_low_pass';
% run 'Scenarios\osv_variable_input_gaussian_random_walk';
run 'Scenarios\osv_variable_input_gaussian_random_walk_low_pass';

% Preallocate arrays
t = 0;
t_array = zeros(1,N+1);    % Time

x_array = zeros(12,N+1);   % States 
x_array(:,1) = x0;

x = x0;

for i=1:N

    % Update state
    switch (integration_method)

        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            [xdot,U,M] = osv(x,u(:,i));
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) osv(x, u(:,i)), t, x, dt); % NEED TO REWRITE
    end

    % Update time
    t = t + dt; 
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    
end

% Plot data
plot_states_path_input(t_array, x_array, u);
