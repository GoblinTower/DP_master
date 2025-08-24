% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

% Select scenario to run
% scenario = 'supply';
% scenario = 'osv';
scenario = 'balchen';

% Run with or without environmental disturbance
disturbance_included = false;

if (strcmp(scenario, 'supply'))

    if (disturbance_included)
        supply_dsr = load('Workspace\supply_dsr_dist.mat');
        supply_dsr_e = load('Workspace\supply_dsr_e_dist.mat');
        supply_pem = load('Workspace\supply_pem_dist.mat');
    else
        supply_dsr = load('Workspace\supply_dsr_no_dist.mat');
        supply_dsr_e = load('Workspace\supply_dsr_e_no_dist.mat');
        supply_pem = load('Workspace\supply_pem_no_dist.mat');
    end
    
    lmf = [supply_dsr, supply_dsr_e, supply_pem];   % Shortcut for List of .Mat Files
    nos = {'DSR', 'DSR\_E', 'PEM' };                % Shortcut for Name Of Simulations
    
    if (disturbance_included)
        storage_path = 'Workspace\Supply\Dist';     % Path where plots are stored
    else
        storage_path = 'Workspace\Supply\No_Dist';
    end

    show_setpoints = true;                          % Show setpoints in plot

    [iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);

elseif (strcmp(scenario, 'osv'))
        
    if (disturbance_included)
        supply_dsr = load('Workspace\osv_dsr_dist.mat');
        supply_dsr_e = load('Workspace\osv_dsr_e_dist.mat');
        supply_pem = load('Workspace\osv_pem_dist.mat');
    else
        supply_dsr = load('Workspace\osv_dsr_no_dist.mat');
        supply_dsr_e = load('Workspace\osv_dsr_e_no_dist.mat');
        supply_pem = load('Workspace\osv_pem_no_dist.mat');
    end
    
    lmf = [supply_dsr, supply_dsr_e, supply_pem];    % Shortcut for List of .Mat Files
    nos = {'DSR', 'DSR\_E', 'PEM' };                 % Shortcut for Name Of Simulations
    
    if (disturbance_included)
        storage_path = 'Workspace\Osv\Dist';         % Path where plots are stored
    else
        storage_path = 'Workspace\Osv\No_Dist';
    end

    show_setpoints = true;                           % Show setpoints in plot

    [iae, tv] = plot_data_osv(lmf, nos, storage_path, show_setpoints);

elseif (strcmp(scenario, 'balchen'))
        
    if (disturbance_included)
        supply_dsr = load('Workspace\balchen_dsr_dist.mat');
        supply_dsr_e = load('Workspace\balchen_dsr_e_dist.mat');
        supply_pem = load('Workspace\balchen_pem_dist.mat');
    else
        supply_dsr = load('Workspace\balchen_dsr_no_dist.mat');
        supply_dsr_e = load('Workspace\balchen_dsr_e_no_dist.mat');
        supply_pem = load('Workspace\balchen_pem_no_dist.mat');
    end
    
    lmf = [supply_dsr, supply_dsr_e, supply_pem];    % Shortcut for List of .Mat Files
    nos = {'DSR', 'DSR\_E', 'PEM' };                 % Shortcut for Name Of Simulations
    
    if (disturbance_included)
        storage_path = 'Workspace\Balchen\Dist';     % Path where plots are stored
    else
        storage_path = 'Workspace\Balchen\No_Dist';
    end

    show_setpoints = true;                           % Show setpoints in plot

    [iae, tv] = plot_data_balchen(lmf, nos, storage_path, show_setpoints);

end

