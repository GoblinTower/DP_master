% Script that runs thruster allocation algorithms

% Clean up workspace
clear, clc, close all;

methods = {'Lagrange (1 tun)', 'Lagrange (2_tun)', 'Lagrange limit (1 tun)', 'Lagrange limit (2_tun)', 'Azimuth'};

external_scenario = 'thruster_allocation';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Thurster allocation %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run explicit lagrange on vessel model with one tunnel
clearvars -except methods external_scenario,                          % First remove previous history
clc, close all force;                                                 % Clear command prompt and close figures
disp(strcat("Running ", methods(1), " on supply model"));
run_one_tunnel = true;
run 'LQ_optimal_control_non_rotable_actuators';

% Run explicit lagrange on vessel model with two tunnels
clearvars -except methods external_scenario,                          % First remove previous history
clc, close all force;                                                 % Clear command prompt and close figures
disp(strcat("Running ", methods(2), " on supply model"));
run_one_tunnel = false;
run 'LQ_optimal_control_non_rotable_actuators';

% Run explicit lagrange on vessel model with one tunnel and limits
clearvars -except methods external_scenario,                          % First remove previous history
clc, close all force;                                                 % Clear command prompt and close figures
disp(strcat("Running ", methods(3), " on supply model"));
run_one_tunnel = true;
run 'LQ_optimal_control_non_rotable_actuators_limit';

% Run explicit lagrange on vessel model with two tunnels and limits
clearvars -except methods external_scenario,                          % First remove previous history
clc, close all force;                                                 % Clear command prompt and close figures
disp(strcat("Running ", methods(4), " on supply model"));
run_one_tunnel = false;
run 'LQ_optimal_control_non_rotable_actuators_limit';

% Run azimuth model
clearvars -except methods external_scenario,                          % First remove previous history
clc, close all force;                                                 % Clear command prompt and close figures
disp(strcat("Running ", methods(5), " on supply model"));
run 'LQ_optimal_control_azimuth';

clear, clc, close all;
