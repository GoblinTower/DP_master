% Simple script for plotting and comparing LQ control simulations on supply
% used in thruster allocation
clear, clc, close all;

addpath("..\..\..\Tools\");

% Simulation number (in Monte Carlo simulation)
simulation_number = 4;

% Simulation type:
% 1: Lagrange 1 tunnel
% 2: Lagrange 2 tunnel
% 3: QP 1 tunnel limit
% 4: QP 2 tunnel limit
% 5: Azimuth
simulation_type = 4;

if (simulation_type == 1)
    sim = load(strcat('Workspace\lq_nonrot_1tunnel_dist_data_', num2str(simulation_number), '.mat')); 
    display_slack_variables = false;
    display_thruster_angles = false;
elseif (simulation_type == 2) 
    sim = load(strcat('Workspace\lq_nonrot_2tunnel_dist_data_', num2str(simulation_number), '.mat')); 
    display_slack_variables = false;
    display_thruster_angles = false;
elseif (simulation_type == 3) 
    sim = load(strcat('Workspace\lq_nonrot_1tunnel_dist_limit_data_', num2str(simulation_number), '.mat')); 
    display_slack_variables = true;
    display_thruster_angles = false;
elseif (simulation_type == 4) 
    sim = load(strcat('Workspace\lq_nonrot_2tunnel_dist_limit_data_', num2str(simulation_number), '.mat')); 
    display_slack_variables = true;
    display_thruster_angles = false;
elseif (simulation_type == 5) 
    sim = load(strcat('Workspace\lq_azimuth_dist_data_', num2str(simulation_number), '.mat')); 
    display_slack_variables = true;
    display_thruster_angles = true;
    angles_to_display = [3, 4];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot thruster forces %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_array = sim.f_array;
number_of_thrusters = size(f_array, 1);

fig = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
t = tiledlayout(number_of_thrusters, 1, "TileSpacing", "compact");

for i=1:number_of_thrusters

    nexttile;
    hold on;
    plot(f_array(i,:), 'r*');
    grid();
    title(strcat(['Forces from thruster: ', num2str(i)]));
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid on, grid minor;
    box on;
    hold off;

end

disp(strcat(['Total value: ', num2str(sum(abs(f_array(:))))]));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot thruster angles %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (display_thruster_angles)

    thr_angle_array = sim.thruster_angles;
    number_of_angles = size(angles_to_display, 2);

    fig = figure('DefaultAxesFontSize', 20);
    t = tiledlayout(1, number_of_angles, "TileSpacing", "compact");

    for i=1:number_of_angles
    
        nexttile;
        hold on;
        plot(rad2deg(thr_angle_array(angles_to_display(i),:)), 'r*');
        grid();
        title(strcat(['Angle of thruster: ', num2str(i)]));
        xlabel('Time [s]');
        ylabel('Thruster angle [deg]');
        grid on, grid minor;
        box on;
        hold off;
    
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot slack variables %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (display_slack_variables)
    
    slack_array = sim.slack_array;

    fig = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
    t = tiledlayout(number_of_thrusters, 1, "TileSpacing", "compact");
    
    for i=1:3
    
        nexttile;
        hold on;
        plot(slack_array(i,:), 'r*');
        grid();
        title(strcat(['Slack variable value: ', num2str(i)]));
        xlabel('Time [s]');
        if (i == 3)
            ylabel('Slack variable [Nm]');
        else
            ylabel('Slack variable [N]');
        end
        grid on, grid minor;
        box on;
        hold off;
    
    end

disp('works')
end
