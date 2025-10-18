% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

lq_stored = load('Workspace\lq_dp_model_stored_matrices_dist_data.mat');
lq_normal = load('Workspace\lq_dp_model_no_stored_matrices_dist_data.mat');
 
lmf = [lq_stored, lq_normal];                               % Shortcut for List of .Mat Files
nos = {'pre-stored matrices', 'normal'};           % Shortcut for Name Of Simulations

storage_folder = 'Workspace\TimeComparison';                  % Path where plots are stored

show_setpoints = true;                                      % Show setpoints in plot

plot_details.path_xlim = [-7,7];
plot_details.path_ylim = [-7,12];

plot_details.force_surge_exponent = 5;
plot_details.force_sway_exponent = 5;
plot_details.momentum_yaw_exponent = 6;

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
model_number_environmental = 2;

[iae, tv] = plot_data_extended(lmf, nos, show_setpoints, 20, storage_folder, plot_details, ...
    show_current_estimate, model_number_environmental);