clear, clc, close all;

addpath("..\..\..\Tools\");

% Load configuration data
run 'setup_osv_variable_input_prbs';

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
            [xdot,U,M] = osv(x, u_array(:,i), 0, 0);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) osv(x, u_array(:,i), 0, 0), t, x, dt);
    end

    % Update time
    t = t + dt; 
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    
end

% Save data for DSR
save Log/dsr_osv_data t_array x_array u_array u_force_array u_squared;

% Plot data
plot_osv_states_path_input(t_array, x_array, u_array);

% Generate State Space Model
if (generate_state_space_model)
    run system_identification.m;
end
