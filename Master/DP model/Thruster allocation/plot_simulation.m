% Simple script for plotting and comparing LQ control simulations on supply
% used in thruster allocation
clear, clc, close all;

addpath("..\..\Tools\");

% Run with or without environmental disturbance
disturbance_included = true;

if (disturbance_included)
    sim = load('Workspace\lq_nonrot_1tunnel_dist_data.mat');         % Does not matter which simulation we run, the force generated is the same, but the force distribution between thrusters varies
else
    sim = load('Workspace\lq_nonrot_1tunnel_no_dist_data');          % Does not matter which simulation we run, the force generated is the same, but the force distribution between thrusters varies
end
    
lmf = [sim];                        % Shortcut for List of .Mat Files
nos = {'LQ supply'};                % Shortcut for Name Of Simulations
    
if (disturbance_included)
    storage_path = 'Workspace\Simulation\Dist';     % Path where plots are stored
else
    storage_path = 'Workspace\Simulation\No_Dist';
end

show_setpoints = true;                                               % Show setpoints in plot

plot_details.path_xlim = [-6,6];
plot_details.path_ylim = [-2,12];

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

show_current_estimate = false;
model_number_environmental = 1;

[iae, tv] = plot_data_extended(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
    show_current_estimate, model_number_environmental);
