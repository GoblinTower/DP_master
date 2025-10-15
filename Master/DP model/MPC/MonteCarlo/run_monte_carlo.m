% Monte Carlo script for MPC simulation
% All simulations are run with external disturbance (weather conditions)
clear, clc, close all;

start_iteration = 1;
number_of_iterations = 100;

external_scenario = 'mc_mpc_optimal_control';

for mc_iteration=start_iteration:number_of_iterations

    % Random seed, varies from 1 - number_of_iterations
    rand_seed = mc_iteration;

    disp(strcat("Running simulation number: ", num2str(mc_iteration)));

    % Run constant psi
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run 'mpc_delta_u_constant_psi.m';                                                 % Run simulation

    % Run constant r
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run 'mpc_delta_u_constant_r.m';                                                   % Run simulation

    % Run LPV
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    close all;                                                                        % Close figures
    run 'mpc_delta_u_lpv';                                                            % Run simulation
end