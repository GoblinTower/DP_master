% Simple script for investigating the results from the Monte Carlo
% simulation
clear, clc, close all;

addpath("..\..\..\Tools\");

number_of_simulations = 100;

% Fetch data from first simulation, should be the same for all
sim1 = load('Workspace\lq_azimuth_dist_data_1.mat');

t_array = sim1.t_array;
number_of_simulation_steps = min(size(t_array,2), size(sim1.u_array,2));

% Preallocate arrays:
f_1_tunnel = zeros(3, number_of_simulation_steps, number_of_simulations);
f_2_tunnel = zeros(4, number_of_simulation_steps, number_of_simulations);

f_1_tunnel_limit = zeros(3, number_of_simulation_steps, number_of_simulations);
f_2_tunnel_limit = zeros(4, number_of_simulation_steps, number_of_simulations);
f_1_tunnel_limit_slack_variable = zeros(3, number_of_simulation_steps, number_of_simulations);
f_2_tunnel_limit_slack_variable = zeros(3, number_of_simulation_steps, number_of_simulations);

f_azimuth = zeros(4, number_of_simulation_steps, number_of_simulations);
f_azimuth_slack_variable = zeros(3, number_of_simulation_steps, number_of_simulations);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Fuel consumption %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% Read data from files
for iteration=1:number_of_simulations
    
    % Get data from 1_tunnel simulations
    sim = load(strcat('Workspace\lq_nonrot_1tunnel_dist_data_', num2str(iteration), '.mat'));
    f_1_tunnel(:,:,iteration) = sim.f_array; 

    % Get data from 2_tunnel simulations
    sim = load(strcat('Workspace\lq_nonrot_2tunnel_dist_data_', num2str(iteration), '.mat'));
    f_2_tunnel(:,:,iteration) = sim.f_array;

    % Get data from 1_tunnel with limit simulations
    sim = load(strcat('Workspace\lq_nonrot_1tunnel_dist_limit_data_', num2str(iteration), '.mat'));
    f_1_tunnel_limit(:,:,iteration) = sim.f_array; 
    f_1_tunnel_limit_slack_variable(:,:,iteration) = sim.slack_array;

    % Get data from 2_tunnel with limit simulations
    sim = load(strcat('Workspace\lq_nonrot_2tunnel_dist_limit_data_', num2str(iteration), '.mat'));
    f_2_tunnel_limit(:,:,iteration) = sim.f_array;
    f_2_tunnel_limit_slack_variable(:,:,iteration) = sim.slack_array;

    % Get data from azimuth simulations
    sim = load(strcat('Workspace\lq_azimuth_dist_data_', num2str(iteration), '.mat'));
    f_azimuth(:,:,iteration) = sim.f_array;
    f_azimuth_slack_variable(:,:,iteration) = sim.slack_array;

end

% Get technical values (TV)
f_total_1_tunnel = squeeze(sum(abs(f_1_tunnel),2));
f_total_2_tunnel = squeeze(sum(abs(f_2_tunnel),2));

f_total_1_tunnel_limit = squeeze(sum(abs(f_1_tunnel_limit),2));
f_total_2_tunnel_limit = squeeze(sum(abs(f_2_tunnel_limit),2));
f_total_1_tunnel_limit_slack = squeeze(sum(abs(f_1_tunnel_limit_slack_variable),2));
f_total_2_tunnel_limit_slack = squeeze(sum(abs(f_2_tunnel_limit_slack_variable),2));

f_total_azimuth = squeeze(sum(abs(f_azimuth),2));
f_total_azimuth_slack = squeeze(sum(abs(f_azimuth_slack_variable), 2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot simulation results %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
t = tiledlayout(3, 1, "TileSpacing", "compact");

%% Lagrange formulation - 1 and 2 tunnel thrusters
nexttile;
hold on;
plot(sum(f_total_1_tunnel(:,:),1), 'r*');
plot(sum(f_total_2_tunnel(:,:),1), 'k*');
grid();
title('Integrated absolute value of force (Lagrange)');
xlabel('Simulation number');
ylabel('IAF');
legend({'1 tunnel', '2 tunnels'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Compute average total value
avg_tv_1_tunnel = mean(f_total_1_tunnel, 2);
disp("Average total value 1 tunnel Lagrange simulations: ");
disp(avg_tv_1_tunnel);
disp("Total:")
disp(sum(avg_tv_1_tunnel));

avg_tv_2_tunnel = mean(f_total_2_tunnel, 2);
disp("Average total value 2 tunnels Lagrange simulations: ");
disp(avg_tv_2_tunnel);
disp("Total:")
disp(sum(avg_tv_2_tunnel));

%% QP formulation - 1 and 2 tunnel thrusters
nexttile;
hold on;
plot(sum(f_total_1_tunnel_limit(:,:),1), 'r*');
plot(sum(f_total_2_tunnel_limit(:,:),1), 'k*');
grid();
title('Integrated absolute value of force (QP)');
xlabel('Simulation number');
ylabel('IAF');
legend({'1 tunnel', '2 tunnels'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Compute average total value
avg_tv_1_tunnel = mean(f_total_1_tunnel_limit, 2);
disp("Average technical value 1 tunnel QP simulations: ");
disp(avg_tv_1_tunnel);
disp("Total:")
disp(sum(avg_tv_1_tunnel));

avg_tv_2_tunnel = mean(f_total_2_tunnel_limit, 2);
disp("Average technical value 2 tunnels QP simulations: ");
disp(avg_tv_2_tunnel);
disp("Total:")
disp(sum(avg_tv_2_tunnel));

%% Non-linear formulation - 1 and 2 tunnel thrusters
nexttile;
hold on;
plot(sum(f_total_azimuth(:,:),1), 'r*');
grid();
title('Integrated absolute value of force (Non-linear)');
xlabel('Simulation number');
ylabel('IAF');
grid on, grid minor;
box on;
hold off;

% Compute average total value
avg_tv_azimuth = mean(f_total_azimuth, 2);
disp("Average technical value for azimuth simulations: ");
disp(avg_tv_azimuth);
disp("Total:");
disp(sum(avg_tv_azimuth));

save_plot(f1, 'mc_results', 'Results\');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot slack variables results %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f2 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
% t = tiledlayout(3, 1, "TileSpacing", "compact");
% 
% nexttile;
% hold on;
% plot(f_total_azimuth_slack(1,:), 'r*');
% grid();
% title('Integrated absolute value of slack variable surge force (Non-linear)');
% xlabel('Simulation number');
% ylabel('Time [s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(f_total_azimuth_slack(2,:), 'r*');
% grid();
% title('Integrated absolute value of slack variable sway force (Non-linear)');
% xlabel('Simulation number');
% ylabel('Time [s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(f_total_azimuth_slack(3,:), 'r*');
% grid();
% title('Integrated absolute value of slack variable yaw momentum (Non-linear)');
% xlabel('Simulation number');
% ylabel('Time [s]');
% grid on, grid minor;
% box on;
% hold off;

f2 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
t = tiledlayout(2, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(sum(f_total_1_tunnel_limit_slack(:,:),1), 'r*');
plot(sum(f_total_2_tunnel_limit_slack(:,:),1), 'k*');
grid();
title('Integrated absolute value of slack variable (QP)');
xlabel('Simulation number');
ylabel('IAS');
legend({'1 tunnel', '2 tunnels'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Compute average IAS QP1
avg_tv_qp1 = mean(f_total_1_tunnel_limit_slack, 2);
disp("IAS for QP1 simulations: ");
disp(avg_tv_qp1);
disp("Total:");
disp(sum(avg_tv_qp1));

% Compute average IAS QP2
avg_tv_qp2 = mean(f_total_2_tunnel_limit_slack, 2);
disp("IAS for QP2 simulations: ");
disp(avg_tv_qp2);
disp("Total:");
disp(sum(avg_tv_qp2));

nexttile;
hold on;
plot(sum(f_total_azimuth_slack(:,:),1), 'r*');
grid();
title('Integrated absolute value of slack variable (Non-linear)');
xlabel('Simulation number');
ylabel('IAS');
grid on, grid minor;
box on;
hold off;

% Compute average IAS azimuth
avg_tv_azimuth = mean(f_total_azimuth_slack, 2);
disp("IAS for azimuth simulations: ");
disp(avg_tv_azimuth);
disp("Total:");
disp(sum(avg_tv_azimuth));

save_plot(f2, 'mc_slack', 'Results\');
