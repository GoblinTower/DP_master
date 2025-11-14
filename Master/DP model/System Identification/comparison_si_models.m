% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");
addpath("plots\");

% Select scenario to run
% scenario = 'none';
% scenario = 'supply';
scenario = 'osv';
% scenario = 'balchen';

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

    % [iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);

    plot_details.path_xlim = [-10,7];
    plot_details.path_ylim = [-10,12];

    plot_details.force_surge_exponent = 6;
    plot_details.force_sway_exponent = 6;
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
    model_number_environmental = 1;
    
    [iae, tv] = plot_si_supply(lmf, nos, show_setpoints, 20, storage_path, plot_details, ...
        show_current_estimate, model_number_environmental);

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
    
    % [iae, tv] = plot_data_balchen(lmf, nos, storage_path, show_setpoints);

    plot_details.path_xlim = [-10,7];
    plot_details.path_ylim = [-10,12];

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
    
    large_plots = true;

    show_current_estimate = true;
    model_number_environmental = 2;
    
    [iae, tv] = plot_si_osv(lmf, nos, show_setpoints, 20, storage_path, plot_details, show_current_estimate, model_number_environmental);

    % The plot_si_supply function does not work appropriately for TV in the case of OSV. So recalculate
    % the true values here
    disp('TV dsr:');
    tv_osv_dsr = sum(abs(lmf(1).u_generalized),2);
    disp(tv_osv_dsr);

    disp('TV dsr_e:');
    tv_osv_dsr_e = sum(abs(lmf(2).u_generalized),2);
    disp(tv_osv_dsr_e);

    disp('TV pem:');
    tv_osv_pem = sum(abs(lmf(3).u_generalized),2);
    disp(tv_osv_pem);

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

    % [iae, tv] = plot_data_balchen(lmf, nos, storage_path, show_setpoints);

    plot_details.path_xlim = [-10,7];
    plot_details.path_ylim = [-10,12];

    plot_details.force_surge_exponent = 6;
    plot_details.force_sway_exponent = 6;
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
    
    large_plots = true;
    
    [iae, tv] = plot_si_balchen(lmf, nos, show_setpoints, 20, storage_path, plot_details, large_plots);

end

% Only valid for simulations with disturbance included
plot_all_paths = true;
if (plot_all_paths)

    details.path_xlim = [-10,12];
    details.path_ylim = [-10,12];

    supply_dsr = load('Workspace\supply_dsr_dist.mat');
    supply_dsr_e = load('Workspace\supply_dsr_e_dist.mat');
    supply_pem = load('Workspace\supply_pem_dist.mat');
    osv_dsr = load('Workspace\osv_dsr_dist.mat');
    osv_dsr_e = load('Workspace\osv_dsr_e_dist.mat');
    osv_pem = load('Workspace\osv_pem_dist.mat');
    balchen_dsr = load('Workspace\balchen_dsr_dist.mat');
    balchen_dsr_e = load('Workspace\balchen_dsr_e_dist.mat');
    balchen_pem = load('Workspace\balchen_pem_dist.mat');

    lmf_supply = [supply_dsr, supply_dsr_e, supply_pem];
    lmf_osv = [osv_dsr, osv_dsr_e, osv_pem];
    lmf_balchen = [balchen_dsr, balchen_dsr, balchen_pem];

    nos = {
            'Supply (DSR)', 'Supply (DSR\_E)', 'Suppy (PEM)', ...
            'OSV (DSR)', 'OSV (DSR\_E)', 'OSV (PEM)', ...
            'Balchen (DSR)', 'Balchen (DSR\_E)', 'Balchen (PEM)' 
          }; 

    number_of_simulations = size(nos, 2);

    figure = figure('DefaultAxesFontSize', 20);
    t = tiledlayout(3,3, "TileSpacing", "compact");

    % Supply
    lmf = lmf_supply;
    for i=1:3

        nexttile;
        hold on;
        plot(lmf(i).x_array(2,:), lmf(i).x_array(1,:))
        plot(lmf(i).x_array(2,1), lmf(i).x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
        plot(lmf(i).x_array(2,end), lmf(i).x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
        grid();
        title(strcat(nos(i)));
        xlabel('East [m]');
        ylabel('North [m]');
        xlim(details.path_xlim);
        ylim(details.path_ylim);
        % legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
        grid on, grid minor;
        box on;
        hold off;
    
    end

    % OSV
    lmf = lmf_osv;
    for i=1:3

        nexttile;
        hold on;
        plot(lmf(i).x_array(8,:), lmf(i).x_array(7,:))
        plot(lmf(i).x_array(8,1), lmf(i).x_array(7,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
        plot(lmf(i).x_array(8,end), lmf(i).x_array(7,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
        grid();
        title(strcat(nos(i+3)));
        xlabel('East [m]');
        ylabel('North [m]');
        xlim(details.path_xlim);
        ylim(details.path_ylim);
        % legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
        grid on, grid minor;
        box on;
        hold off;
    
    end

    % Balchen    
    lmf = lmf_balchen;
    for i=1:3
    
        nexttile;
        hold on;
        plot(lmf(i).x_array(2,:), lmf(i).x_array(1,:))
        plot(lmf(i).x_array(2,1), lmf(i).x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
        plot(lmf(i).x_array(2,end), lmf(i).x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
        grid();
        title(strcat(nos(i+6)));
        xlabel('East [m]');
        ylabel('North [m]');
        xlim(details.path_xlim);
        ylim(details.path_ylim);
        % legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
        grid on, grid minor;
        box on;
        hold off;
    
    end
    
    save_plot(figure, 'path', 'Workspace\AllPaths');
end