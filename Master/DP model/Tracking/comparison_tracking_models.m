% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

% Run with or without environmental disturbance
simple_tracking = true;
disturbance_included = false;

if (simple_tracking)
    if (disturbance_included)
        lpv = load('Workspace\lpv_simple_tracking_with_dist_data.mat');
        lq = load('Workspace\lq_simple_tracking_with_dist_data');
    else
        lpv = load('Workspace\lpv_simple_tracking_without_dist_data.mat');
        lq = load('Workspace\lq_simple_tracking_without_dist_data.mat');
    end
else
    if (disturbance_included)
        lpv = load('Workspace\lpv_trajectory_tracking_with_dist_data.mat');
        lq = load('Workspace\lq_trajectory_tracking_with_dist_data.mat');
    else
        lpv = load('Workspace\Workspace\lpv_trajectory_tracking_without_dist_data.mat');
        lq = load('Workspace\lq_trajectory_tracking_without_dist_data.mat');
    end
end
 
lmf = [lpv, lq];                                            % Shortcut for List of .Mat Files
nos = {'LPV', 'LQ' };                                       % Shortcut for Name Of Simulations

if (disturbance_included)
    storage_path = 'Workspace\Dist';                        % Path where plots are stored
else
    storage_path = 'Workspace\No_Dist';
end

show_setpoints = true;                                      % Show setpoints in plot

[iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);
