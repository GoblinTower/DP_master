% Monte Carlo script for running GreenDP
clear, clc, close all;

number_of_iterations = 25;

external_scenario = 'compare_time';

time_array_stored_matrices = zeros(1, number_of_iterations);
time_array_continuously_calculated = zeros(1, number_of_iterations);

for mc_iteration=1:number_of_iterations

    % Run scenario with stored matrices
    clearvars -except number_of_iterations mc_iteration external_scenario ...
        time_array_stored_matrices time_array_continuously_calculated                 % Remove history from previous simulation
    clc, close all;                                                                   % Clear command prompt and close figures
    disp(strcat("Running stored matrix simulation number: ", num2str(mc_iteration)));
    run_stored_matrices = true;                                                       % Define whether to use GreenDP mode
    run 'LQ_dp_model_optimal_control_stored_matrices.m';                              % Run simulation
    time_array_stored_matrices(mc_iteration) = time_elapsed;                          % Store running time from simulation 

    % Run scenario without stored matrices
    clearvars -except number_of_iterations mc_iteration external_scenario ...
        time_array_stored_matrices time_array_continuously_calculated                 % Remove history from previous simulation
    clc, close all;                                                                   % Clear command prompt and close figures
    disp(strcat("Running normal simulation number: ", num2str(mc_iteration)));
    run_stored_matrices = false;                                                      % Define whether to use GreenDP mode
    run 'LQ_dp_model_optimal_control_stored_matrices.m';                              % Run simulation
    time_array_continuously_calculated(mc_iteration) = time_elapsed;                  % Store running time from simulation 

end

save("ComparisonTime\time_measured.mat", "time_array_stored_matrices", "time_array_continuously_calculated");