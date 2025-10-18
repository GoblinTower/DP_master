% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

% Run with or without environmental disturbance
simple_tracking = true;
disturbance_included = true;

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
        lpv = load('Workspace\lpv_trajectory_tracking_without_dist_data.mat');
        lq = load('Workspace\lq_trajectory_tracking_without_dist_data.mat');
    end
end
 
lmf = [lpv, lq];                                            % Shortcut for List of .Mat Files
nos = {'LPV', 'LQ' };                                       % Shortcut for Name Of Simulations

if (simple_tracking)
    if (disturbance_included)
        storage_path = 'Workspace\Simple_Dist';             % Path where plots are stored
    else
        storage_path = 'Workspace\Simple_No_Dist';          % Path where plots are stored
    end
else
    if (disturbance_included)
        storage_path = 'Workspace\Advanded_Dist';           % Path where plots are stored
    else
        storage_path = 'Workspace\Advanced_No_Dist';        % Path where plots are stored
    end
end

show_setpoints = true;                                     % Show setpoints in plot

if (simple_tracking)
    plot_details.path_xlim = [-30,340];
    plot_details.path_ylim = [-30,490];
else
    plot_details.path_xlim = [-10,90];
    plot_details.path_ylim = [-30,50];
end

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

show_current_estimate = false;
large_plots = true;

if (simple_tracking)
    [iae, tv] = plot_data_extended_with_waypoints(lmf, nos, show_setpoints, 20, storage_path, plot_details, show_current_estimate, large_plots);
else
    show_current_estimate = false;
    model_number_environmental = 1;
    [iae, tv] = plot_data_extended_cubic_spline(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
        show_current_estimate, model_number_environmental);
end