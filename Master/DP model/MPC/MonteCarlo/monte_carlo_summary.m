% Simple script for investigating the results from the Monte Carlo
% simulation
clear, clc, close all;

addpath("..\..\..\Tools\");

number_of_simulations = 100;

% Fetch data from first simulation, should be the same for all
sim1 = load('Workspace\mpc_const_psi_du_with_dist_1.mat');

t_array = sim1.t_array;
x_array = sim1.x_array;
setpoint_array = sim1.setpoint;

% Time when setpoint changes occur
time_of_heading_changes = find(diff(setpoint_array(3,:)))+1;

number_of_simulation_steps = min(size(t_array,2), size(x_array,2)); %, size(setpoint_array,2));
% number_of_simulation_steps = 50;

% Preallocate arrays:
error_mpc_psi_disturbance = zeros(3, number_of_simulation_steps, number_of_simulations);
error_mpc_r_disturbance = zeros(3, number_of_simulation_steps, number_of_simulations);
error_mpc_lpv_disturbance = zeros(3, number_of_simulation_steps, number_of_simulations);
setpoint_array_common = zeros(3, number_of_simulation_steps, number_of_simulations);

%%%%%%%%%%%%%%%%%%%%%%
%%% Setpoint error %%%
%%%%%%%%%%%%%%%%%%%%%%

% Read data from files
for iteration=1:number_of_simulations
    
    % Get data from constant psi MPC
    sim = load(strcat('Workspace\mpc_const_psi_du_with_dist_', num2str(iteration), '.mat'));
    error_mpc_psi_disturbance(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps) - sim.x_array(1:3,1:number_of_simulation_steps); 
    setpoint_array_common(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps);

    % Get data from constant r MPC
    sim = load(strcat('Workspace\mpc_const_r_du_with_dist_', num2str(iteration), '.mat'));
    error_mpc_r_disturbance(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps) - sim.x_array(1:3,1:number_of_simulation_steps); 

    % Get data from constant LPV MPC
    sim = load(strcat('Workspace\mpc_lpv_du_with_dist_', num2str(iteration), '.mat'));
    error_mpc_lpv_disturbance(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps) - sim.x_array(1:3,1:number_of_simulation_steps); 
end

% Get RMSE
error_mpc_psi_disturbance_rmse = squeeze(sqrt(sum(error_mpc_psi_disturbance.^2,2)/number_of_simulation_steps));
error_mpc_r_disturbance_rmse = squeeze(sqrt(sum(error_mpc_r_disturbance.^2,2)/number_of_simulation_steps));
error_mpc_lpv_disturbance_rmse = squeeze(sqrt(sum(error_mpc_lpv_disturbance.^2,2)/number_of_simulation_steps));

% Get Integrated Absolute Error (IAE)
error_mpc_psi_disturbance_iae = squeeze(sum(abs(error_mpc_psi_disturbance),2));
error_mpc_r_disturbance_iae = squeeze(sum(abs(error_mpc_r_disturbance),2));
error_mpc_lpv_disturbance_iae = squeeze(sum(abs(error_mpc_lpv_disturbance),2));

% Calculate total change in heading
heading_changes = squeeze(setpoint_array_common(3, [1, time_of_heading_changes], :));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot simulation results %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
t = tiledlayout(3, 2, "TileSpacing", "compact");

%% RMSE MPC with constant psi with disturbance
nexttile;
hold on;
plot(error_mpc_psi_disturbance_rmse(1,:));
plot(error_mpc_psi_disturbance_rmse(2,:));
plot(error_mpc_psi_disturbance_rmse(3,:));
grid();
title('RMSE in MPC (const \psi)');
xlabel('Simulation number');
ylabel('RMSE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

rmse_psi_mean_disturbance = mean(error_mpc_psi_disturbance_rmse, 2);
disp("RMSE value for MPC constant psi simulations with disturbance: ");
disp(rmse_psi_mean_disturbance);
disp("Total:");
disp(sum(rmse_psi_mean_disturbance));

%% IAE MPC with constant psi with disturbance
nexttile;
hold on;
plot(error_mpc_psi_disturbance_iae(1,:));
plot(error_mpc_psi_disturbance_iae(2,:));
plot(error_mpc_psi_disturbance_iae(3,:));
grid();
title('IAE in MPC (const \psi)');
xlabel('Simulation number');
ylabel('IAE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

iae_psi_mean_disturbance = mean(error_mpc_psi_disturbance_iae, 2);
disp("IAE value for MPC constant psi simulations with disturbance: ");
disp(iae_psi_mean_disturbance);
disp("Total:");
disp(sum(iae_psi_mean_disturbance));

%% RMSE MPC with constant r with disturbance
nexttile;
hold on;
plot(error_mpc_r_disturbance_rmse(1,:));
plot(error_mpc_r_disturbance_rmse(2,:));
plot(error_mpc_r_disturbance_rmse(3,:));
grid();
title('RMSE MPC (const r)');
xlabel('Simulation number');
ylabel('RMSE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

rmse_r_mean_disturbance = mean(error_mpc_r_disturbance_rmse, 2);
disp("RMSE value for MPC constant r simulations with disturbance: ");
disp(rmse_r_mean_disturbance);
disp("Total:");
disp(sum(rmse_r_mean_disturbance));

%% IAE MPC with constant r with disturbance
nexttile;
hold on;
plot(error_mpc_r_disturbance_iae(1,:));
plot(error_mpc_r_disturbance_iae(2,:));
plot(error_mpc_r_disturbance_iae(3,:));
grid();
title('IAE MPC (const r)');
xlabel('Simulation number');
ylabel('IAE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

iae_r_mean_disturbance = mean(error_mpc_r_disturbance_iae, 2);
disp("IAE value for MPC constant r simulations with disturbance: ");
disp(iae_r_mean_disturbance);
disp("Total:");
disp(sum(iae_r_mean_disturbance))

%% RMSE MPC LPV with disturbance
nexttile;
hold on;
plot(error_mpc_lpv_disturbance_rmse(1,:));
plot(error_mpc_lpv_disturbance_rmse(2,:));
plot(error_mpc_lpv_disturbance_rmse(3,:));
grid();
title('RMSE in MPC (LPV)');
xlabel('Simulation number');
ylabel('RMSE');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

rmse_lpv_mean_disturbance = mean(error_mpc_lpv_disturbance_rmse, 2);
disp("RMSE value for LPV MPC simulations with disturbance: ");
disp(rmse_lpv_mean_disturbance);
disp("Total:");
disp(sum(rmse_lpv_mean_disturbance));

%% IAE LPV MPC with disturbance
nexttile;
hold on;
plot(error_mpc_lpv_disturbance_iae(1,:));
plot(error_mpc_lpv_disturbance_iae(2,:));
plot(error_mpc_lpv_disturbance_iae(3,:));
grid();
title('IAE in MPC (LPV)');
xlabel('Simulation number');
ylabel('IAE');
legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

iae_lpv_mean_disturbance = mean(error_mpc_lpv_disturbance_iae, 2);
disp("IAE value for LPV MPC simulations with disturbance: ");
disp(iae_lpv_mean_disturbance);
disp("Total:");
disp(sum(iae_lpv_mean_disturbance))

save_plot(f1, 'mc_results', 'Results\');

% Display total heading change
total_heading_change_array = zeros(1,number_of_simulations);
for i=1:number_of_simulations
    total_heading_change_array(1,i) = sum(get_smallest_angle_differences(heading_changes(:,i)));
end

f2 = figure('DefaultAxesFontSize', 20);
t = tiledlayout(1, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(rad2deg(total_heading_change_array));
grid();
title('Total heading setpoint change');
xlabel('Simulation number');
ylabel('Total heading change [°]');
grid on, grid minor;
box on;
hold off;

save_plot(f2, 'mc_results_heading_change', 'Results\');

function diff = get_smallest_angle_differences(angle_array)
    % Assumes input is in radians
    % Returns an array of input differences

    % Normalized angles
    angle_array_normalized = mod(angle_array, 2*pi);
  
    % Number of angles:
    number_of_angles = length(angle_array);

    % Preallocate return array
    diff = zeros(1, number_of_angles-1);

    % Compute array differnces
    for i=1:(number_of_angles - 1)
        case1 = abs(angle_array_normalized(i+1) - angle_array_normalized(i));
        case2 = 2*pi - case1;
        diff(i) = min(case1, case2);
    end
end
