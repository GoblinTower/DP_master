% Simple script for plotting and comparing LQ control simulations
clear, clc, close all;

addpath("..\..\Tools\");

% Select scenario to run
% Circle: Current changes 360 degrees during 800 seconds run
% scenario = 'circle';
% scenario = 'drift';
scenario = 'mild';

if (strcmp(scenario, 'circle'))

    % Load data from circle scenario
    normal_nmpc = load('Workspace\nmpc_no_green_dp_dist_circle_data.mat');
    green_dp = load('Workspace\nmpc_green_dp_dist_circle_data.mat');
    
    lmf = [normal_nmpc, green_dp];                  % Shortcut for List of .Mat Files
    nos = {'Normal NMPC', 'Green DP'};              % Shortcut for Name Of Simulations
    
    storage_path = 'Workspace\Circle';              % Path where plots are stored
    
    show_setpoints = false;                         % Show setpoints in plot

elseif (strcmp(scenario, 'drift'))
        
    % Load data from drift scenario
    normal_nmpc = load('Workspace\nmpc_no_green_dp_dist_drift_data.mat');
    green_dp = load('Workspace\nmpc_green_dp_dist_drift_data.mat');
    
    lmf = [normal_nmpc, green_dp];                  % Shortcut for List of .Mat Files
    nos = {'Normal NMPC', 'Green DP'};              % Shortcut for Name Of Simulations
    
    storage_path = 'Workspace\Drift';               % Path where plots are stored
    
    show_setpoints = false;                         % Show setpoints in plot

elseif (strcmp(scenario, 'mild'))
        
    % Load data from drift scenario
    normal_nmpc = load('Workspace\nmpc_no_green_dp_dist_mild_data.mat');
    green_dp = load('Workspace\nmpc_green_dp_dist_mild_data.mat');
    
    lmf = [normal_nmpc, green_dp];                  % Shortcut for List of .Mat Files
    nos = {'Normal NMPC', 'Green DP'};              % Shortcut for Name Of Simulations
    
    storage_path = 'Workspace\Drift';               % Path where plots are stored
    
    show_setpoints = false;                         % Show setpoints in plot

end

[iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);