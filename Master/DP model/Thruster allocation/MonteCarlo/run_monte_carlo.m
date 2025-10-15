% Monte Carlo script for thruster allocation
clear, clc, close all;

start_iteration = 1;
number_of_iterations = 100;

external_scenario = 'mc_thruster_allocation';

for mc_iteration=start_iteration:number_of_iterations

    % Random seed, varies from 1 - number_of_iterations
    rand_seed = mc_iteration;

    disp(strcat("Running simulation number: ", num2str(mc_iteration)));

    % Run 1 tunnel thruster without limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_one_tunnel = true;                                                            % Tunnel thruster flag
    run 'LQ_optimal_control_non_rotable_actuators.m';                                 % Run simulation

    % Run 2 tunnel thruster without limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_one_tunnel = false;                                                           % Tunnel thruster flag
    run 'LQ_optimal_control_non_rotable_actuators.m';                                 % Run simulation

    % Run 1 tunnel thruster with limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_one_tunnel = true;                                                            % Tunnel thruster flag
    run 'LQ_optimal_control_non_rotable_actuators_limit.m';                           % Run simulation

    % Run 2 tunnel thruster with limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_one_tunnel = false;                                                           % Tunnel thruster flag
    run 'LQ_optimal_control_non_rotable_actuators_limit.m';                           % Run simulation

    % Run azimuth setup
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run 'LQ_optimal_control_azimuth.m';                                               % Run simulation

end