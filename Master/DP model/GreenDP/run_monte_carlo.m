% Monte Carlo script for running GreenDP
clear, clc, close all;

start_iteration = 1;
number_of_iterations = 100;

external_scenario = 'mc_green_dp';

for mc_iteration=start_iteration:number_of_iterations

    % Random seed, varies from 1 - number_of_iterations
    rand_seed = mc_iteration;

    % Run scenario with greenDP
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history
    clc, close all;                                                                   % Clear command prompt and close figures
    disp(strcat("Running greenDP simulation number: ", num2str(mc_iteration)));
    run_greenDP = true;                                                               % Define whether to use GreenDP mode
    run 'nmpc_supply_model_greenDP.m';                                                % Run simulation

    % Run scenario without greenDP
    clearvars -except number_of_iterations rand_seed mc_iteration external_scenario   % First remove previous history 
    clc, close all;                                                                   % Clear command prompt and close figures
    disp(strcat("Running normal simulation number: ", num2str(mc_iteration)));
    run_greenDP = false;                                                              % Define whether to use GreenDP mode
    run 'nmpc_supply_model_greenDP.m';                                                % Run simulation

end