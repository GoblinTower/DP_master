% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");
addpath("plots\");

% Load data
% Strategy 2
supply_str2 = load('Workspace\supply_int_mode4_no_dist.mat');

% Strategy 3
supply_str3 = load('Workspace\supply_int_mode2_no_dist.mat');


    
run_strategy = 2;

if (run_strategy == 2)
    lmf = [supply_str2];                 % Shortcut for List of .Mat Files
    nos = {'Strategy 2'};                % Shortcut for Name Of Simulations
    storage_path = 'Workspace/DoubleIntegratorSimulations/Str2';
elseif (run_strategy == 3)
    lmf = [supply_str3];                 % Shortcut for List of .Mat Files
    nos = {'Strategy 3'};                % Shortcut for Name Of Simulations
    storage_path = 'Workspace/DoubleIntegratorSimulations/Str3';
end

show_setpoints = true;                             % Show setpoints in plot

plot_details.path_xlim = [-10,7];
plot_details.path_ylim = [-10,12];

plot_details.force_surge_exponent = 6;
plot_details.force_sway_exponent = 6;
plot_details.momentum_yaw_exponent = 8;

plot_details.kalman_exponent = 5;

plot_details.wind_surge_exponent = 4;
plot_details.wind_sway_exponent = 4;
plot_details.wind_momentum_exponent = 5;

plot_details.current_north_exponent = 5;
plot_details.current_east_exponent = 5;

plot_details.wave_north_exponent = 4;
plot_details.wave_east_exponent = 4;
plot_details.wave_momentum_exponent = 3;

show_current_estimate = true;
model_number_environmental = 1;

[iae, tv] = plot_si_supply(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
    show_current_estimate, model_number_environmental);

