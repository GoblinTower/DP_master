% Simple script for plotting and comparing LQ control simulations on supply
% used in thruster allocation
clear, clc, close all;

addpath("..\..\Tools\");

% Select simulation
load_simulation_number = 6;

% Load simulation
sim_psi = load(strcat('Workspace\mpc_const_psi_du_with_dist_', num2str(load_simulation_number), '.mat'));
sim_r  = load(strcat('Workspace\mpc_const_r_du_with_dist_', num2str(load_simulation_number), '.mat'));   
sim_lpv  = load(strcat('Workspace\mpc_lpv_du_with_dist_', num2str(load_simulation_number), '.mat'));

lmf = [sim_psi, sim_r, sim_lpv];                                               % Shortcut for List of .Mat Files
nos = {
           'Const \psi',
           'Const r',
           'LPV'
       };                                                                      % Shortcut for Name Of Simulations
    
storage_path = 'Workspace';         % Path where plots are stored

show_setpoints = true;                                               % Show setpoints in plot

plot_details.path_xlim = [-20,20];
plot_details.path_ylim = [-20,20];

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

[iae, tv] = plot_data_extended(lmf, nos, show_setpoints, 20, storage_path, plot_details, show_current_estimate);
