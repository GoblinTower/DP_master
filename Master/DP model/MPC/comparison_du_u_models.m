% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

% Run with or without environmental disturbance
disturbance_included = true;
use_du_formulation = true;

if (use_du_formulation)
    if (disturbance_included)
        mpc_const_psi = load('Workspace\mpc_const_psi_du_with_dist.mat');
        mpc_const_r = load('Workspace\mpc_const_r_du_with_dist.mat');
        lpv = load('Workspace\mpc_lpv_du_with_dist.mat');
        body = load('Workspace\mpc_recalculate_setpoint_du_with_dist.mat');
    else
        mpc_const_psi = load('Workspace\mpc_const_psi_du_without_dist.mat');
        mpc_const_r = load('Workspace\mpc_const_r_du_without_dist.mat');
        lpv = load('Workspace\mpc_lpv_du_without_dist.mat');
        body = load('Workspace\mpc_recalculate_setpoint_du_without_dist.mat');
    end
else
    if (disturbance_included)
        mpc_const_psi = load('Workspace\mpc_const_psi_u_with_dist.mat');
        mpc_const_r = load('Workspace\mpc_const_r_u_with_dist.mat');
        lpv = load('Workspace\mpc_lpv_u_with_dist.mat');
    else
        mpc_const_psi = load('Workspace\mpc_const_psi_u_without_dist.mat');
        mpc_const_r = load('Workspace\mpc_const_r_u_without_dist.mat');
        lpv = load('Workspace\mpc_lpv_u_without_dist.mat');
    end
end
 
% lmf = [mpc_const_psi, mpc_const_r, lpv, body];              % Shortcut for List of .Mat Files
% nos = {'Constant \psi', 'Constant r', 'LPV', 'Body' };      % Shortcut for Name Of Simulations

lmf = [mpc_const_psi, mpc_const_r, lpv];                      % Shortcut for List of .Mat Files
nos = {'\psi', 'r', 'LPV'};                                   % Shortcut for Name Of Simulations

if (disturbance_included)
    storage_path = 'Workspace\Dist';                          % Path where plots are stored
else
    storage_path = 'Workspace\No_Dist';
end

show_setpoints = true;                                      % Show setpoints in plot

% [iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);

show_setpoints = true;                                      % Show setpoints in plot

plot_details.path_xlim = [-10,7];
plot_details.path_ylim = [-10,12];

plot_details.force_surge_exponent = 6;
plot_details.force_sway_exponent = 6;
plot_details.momentum_yaw_exponent = 8;

plot_details.kalman_exponent = 4;

plot_details.wind_surge_exponent = 4;
plot_details.wind_sway_exponent = 4;
plot_details.wind_momentum_exponent = 5;

plot_details.current_north_exponent = 5;
plot_details.current_east_exponent = 5;

plot_details.wave_north_exponent = 4;
plot_details.wave_east_exponent = 4;
plot_details.wave_momentum_exponent = 3;

show_current_estimate = false;
model_number_environmental = 2;

[iae, tv] = plot_data_extended_large(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
    show_current_estimate, model_number_environmental);
