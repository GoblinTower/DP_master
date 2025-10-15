% Simple script for plotting results from NMPC.
clear, clc, close all;

disturbance_included = true;

% Load data
if (disturbance_included)
    dp_du = load('Workspace\nmpc_du_dist_data.mat');
    nos = {'NMPC'};
    storage_path = 'Workspace\Dist';
else
    dp_du = load('Workspace\nmpc_du_no_dist_data.mat');
    nos = {'NMPC (no disturbance)'};
    storage_path = 'Workspace\No_Dist';
end

lmf = [dp_du];

show_setpoints = true;                                      % Show setpoints in plot

plot_details.path_xlim = [-6,13];
plot_details.path_ylim = [-6,6];

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

[iae, tv] = plot_data_extended(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
    show_current_estimate, model_number_environmental);