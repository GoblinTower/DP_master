% Script for performing system identification based on the 
% input and output data from a vessel simulation, in this case
% simulated using the supply model from MSS (Marine Systems Simulator)
% toolbox.

addpath("..\..\..\Tools\");

if (exist('external_scenario', 'var'))
    run 'setup_supply_variable_input_prbs';
else
    % This section represents code used when running this script directly
    clear, clc, close all;

    % Load configuration data
    run 'setup_supply_variable_input_prbs';
end

% Preallocate arrays
t = 0;
t_array = zeros(1,N+1);    % Time

x_array = zeros(6,N+1);   % States 
x_array(:,1) = x0;

x = x0;

for i=1:N

    % Update state
    switch (integration_method)

        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            [xdot,U,M] = supply(x,u_array(:,i));
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) supply(x, u_array(:,i)), t, x, dt);
    end

    % Update time
    t = t + dt; 
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    
end

% Save data for DSR
save_path = strcat('Log\integrator_supply_data');
save(save_path, 't_array', 'x_array', 'u_array');

% Plot data
plot_supply_states_path_input(t_array, x_array, u_array);

% Generate State Space Model
if (generate_state_space_model)
    run integrator_system_identification.m;
end
