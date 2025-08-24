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
 
lmf = [mpc_const_psi, mpc_const_r, lpv, body];              % Shortcut for List of .Mat Files
nos = {'Constant \psi', 'Constant r', 'LPV', 'Body' };      % Shortcut for Name Of Simulations

if (disturbance_included)
    storage_path = 'Workspace\Dist';                        % Path where plots are stored
else
    storage_path = 'Workspace\No_Dist';
end

show_setpoints = true;                                      % Show setpoints in plot

[iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);
