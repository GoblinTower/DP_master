% Monte Carlo script for NMPC simulation
% All simulations are run with external disturbance (weather conditions)
clear, clc, close all;

start_iteration = 65;
number_of_iterations = 100;

external_scenario = 'mc_nmpc_control';

for mc_iteration=start_iteration:number_of_iterations

    % Random seed, varies from 1 - number_of_iterations
    rand_seed = mc_iteration;

    disp(strcat("Running simulation number: ", num2str(mc_iteration)));

    % Run NMPC
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run 'nmpc_du_linear_kalman.m';                                                 % Run simulation
end