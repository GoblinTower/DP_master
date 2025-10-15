% Monte Carlo script for thruster allocation
clear, clc, close all;

start_iteration = 1;
number_of_iterations = 100;

external_scenario = 'mc_lq_optimal_control';

for mc_iteration=start_iteration:number_of_iterations

    % Random seed, varies from 1 - number_of_iterations
    rand_seed = mc_iteration;

    disp(strcat("Running simulation number: ", num2str(mc_iteration)));

    % Run 1 tunnel thruster without limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_with_disturbance = true;                                                      % Environmental disturbance flag
    run 'LQ_dp_model_optimal_control.m';                                              % Run simulation

    % Run 2 tunnel thruster without limits
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run_with_disturbance = false;                                                     % Environmental disturbance flag
    run 'LQ_dp_model_optimal_control.m';                                              % Run simulation
end