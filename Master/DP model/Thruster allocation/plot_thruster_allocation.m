% Simple script for plotting thruster allocation
clear, clc, close all;

addpath(" ..\..\Tools\");

time_array = [110, 610, 950]; 

% Settings
font_size = 20;
scale_forces = 3e4;
scale_surge = 3e4;
scale_sway = 3e4;
scale_momentum = 1e5;
pause_between_plotting = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%% Explicit Lagrange 1_tunnel %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load data
sim = load('Workspace\lq_nonrot_1tunnel_dist_data.mat'); 

% Folder to store snapshots
folder = 'Snapshots';
filename = 'Lagrange_1_tun';

number_of_time_samples = length(time_array);

fig = plot_thrusters(sim, time_array, font_size, 70, 8, scale_forces, scale_surge, scale_sway, scale_momentum);
save_plot(fig, filename, folder);

fig2 = plot_non_rotatable_setup(sim, 20);
save_plot(fig2, strcat(filename, '_forces'), 'Forces')

calculate_integrated_absolute_force_value(sim, filename);

if (pause_between_plotting)
    pause;
end

close(fig), close (fig2); %, clearvars -except time_array font_size scale_forces scale_surge scale_sway scale_momentum pause_between_plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Explicit Lagrange 2_tunnel %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load data
sim = load('Workspace\lq_nonrot_2tunnel_dist_data.mat'); 

% Folder to store snapshots
folder = 'Snapshots';
filename = 'Lagrange_2_tun';

number_of_time_samples = length(time_array);

fig = plot_thrusters(sim, time_array, font_size, 70, 8, scale_forces, scale_surge, scale_sway, scale_momentum);
save_plot(fig, filename, folder);

fig2 = plot_non_rotatable_setup(sim, 20);
save_plot(fig2, strcat(filename, '_forces'), 'Forces')

calculate_integrated_absolute_force_value(sim, filename);

if (pause_between_plotting)
    pause;
end 

close(fig), close(fig2) %, clearvars -except time_array font_size scale_forces scale_surge scale_sway scale_momentum pause_between_plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Explicit Lagrange 1_tunnel limit %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load data
sim = load('Workspace\lq_nonrot_1tunnel_dist_limit_data.mat'); 

% Folder to store snapshots
folder = 'Snapshots';
filename = 'Lagrange_1_tun_limit';

number_of_time_samples = length(time_array);

fig = plot_thrusters(sim, time_array, font_size, 70, 8, scale_forces, scale_surge, scale_sway, scale_momentum);
save_plot(fig, filename, folder);

fig2 = plot_non_rotatable_setup(sim, 20);
save_plot(fig2, strcat(filename, '_forces'), 'Forces')

calculate_integrated_absolute_force_value(sim, filename);

if (pause_between_plotting)
    pause;
end

close(fig), close(fig2); %, clearvars -except time_array font_size scale_forces scale_surge scale_sway scale_momentum pause_between_plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Explicit Lagrange 2 tunnel limit %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load data
sim = load('Workspace\lq_nonrot_2tunnel_dist_limit_data.mat'); 

% Folder to store snapshots
folder = 'Snapshots';
filename = 'Lagrange_2_tun_limit';

number_of_time_samples = length(time_array);

fig = plot_thrusters(sim, time_array, font_size, 70, 8, scale_forces, scale_surge, scale_sway, scale_momentum);
save_plot(fig, filename, folder);

fig2 = plot_non_rotatable_setup(sim, 20);
save_plot(fig2, strcat(filename, '_forces'), 'Forces')

calculate_integrated_absolute_force_value(sim, filename);

if (pause_between_plotting)
    pause;
end

close(fig), close(fig2); %, clearvars -except time_array font_size scale_forces scale_surge scale_sway scale_momentum pause_between_plotting
%%%%%%%%%%%%%%%
%% Azimuth %%%%
%%%%%%%%%%%%%%%
% Load data
sim = load('Workspace\lq_azimuth_dist_data.mat');  

% Folder to store snapshots
folder = 'Snapshots';
filename = 'Azimuth';

number_of_time_samples = length(time_array);

azi = [3,4];
fig = plot_thrusters_with_azimuths(sim, time_array, font_size, 70, 8, scale_forces, scale_surge, scale_sway, scale_momentum, azi);
save_plot(fig, filename, folder);

fig2 = plot_azimuth_setup(sim, 20);
save_plot(fig2, strcat(filename, '_forces'), 'Forces')

calculate_integrated_absolute_force_value(sim, filename);

if (pause_between_plotting)
    pause;
end

close(fig), close(fig2)
clear, clc, close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Write out thruster usage metric %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fig = plot_non_rotatable_setup(ws, font_size)

    length = min(size(ws.f_array, 2), size(ws.t_array, 2));
    number_of_thrusters = size(ws.thruster_names, 2);

    dp = get(groot, 'DefaultFigurePosition');   % Default position

    % Create plot
    fig = figure('DefaultAxesFontSize', font_size, 'Position', [dp(1), dp(2), dp(3), 0.7*dp(4)]);
    t = tiledlayout(1, 1, "TileSpacing", "compact");

    % Thruster force
    nexttile;
    hold on;
    for i=1:number_of_thrusters
        plot(ws.t_array(1:length), ws.f_array(i,1:length));
    end
    grid();
    title('Thruster forces');
    xlabel('t [s]');
    ylabel('Force [N]');
    legend(ws.thruster_names, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    hold off;

    % save_plot(fig, filename, folder);

end

function fig = plot_azimuth_setup(ws, font_size)

    length = min(size(ws.f_array, 2), size(ws.t_array, 2));
    number_of_thrusters = size(ws.thruster_names, 2);
    number_of_angles = size(ws.angle_names, 2);

    % Create plot
    fig = figure('DefaultAxesFontSize', font_size);
    t = tiledlayout(2, 1 , "TileSpacing", "compact");

    % Thruster force
    nexttile;
    hold on;
    for i=1:number_of_thrusters
        plot(ws.t_array(1:length), ws.f_array(i,1:length));
    end
    grid();
    title('Thruster forces');
    xlabel('t [s]');
    ylabel('Force [N]');
    legend(ws.thruster_names, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    hold off;

    % Angles
    nexttile;
    hold on;
    for i=1:number_of_angles
        % plot(ws.t_array(1:length), rad2deg(convert_angle(ws.angle_array(i,1:length))));
        plot(ws.t_array(1:length), rad2deg(ws.angle_array(i,1:length)));
    end
    grid();
    title('Thruster angles');
    xlabel('t [s]');
    ylabel('Deg [°]');
    legend(ws.angle_names, 'Location', 'Best');
    grid on, grid minor;
    box on;
    hold off;

    % save_plot(fig, filename, folder);

end

function new_angle = convert_angle(angle)
    mod_angle = mod(angle, 2*pi);

    if (mod_angle <= pi)
        % Do nothing
    else
        mod_angle = mod_angle - pi;
    end

    new_angle = mod_angle;
end

function [iaf total] = calculate_integrated_absolute_force_value(ws, name)

    iaf = sum(abs(ws.f_array),2);
    total = sum(iaf);
    
    disp(strcat("Integrated absolute force (", name, ": "));
    disp("IAF");
    disp(iaf);
    disp("Total");
    disp(total);

end
