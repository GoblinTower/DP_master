% Simple script for investigating the results from the Monte Carlo
% simulation
clear, clc, close all;

addpath("..\..\..\Tools\");

number_of_simulations = 100;

% Fetch data from first simulation, should be the same for all
sim1 = load('Workspace\lq_dp_model_dist_data_1.mat');

t_array = sim1.t_array;
x_array = sim1.x_array;
setpoint_array = sim1.setpoint;

% Time when setpoint changes occur
time_of_heading_changes = find(diff(setpoint_array(3,:)))+1;

number_of_simulation_steps = min(size(t_array,2), size(x_array,2)); %, size(setpoint_array,2));
% number_of_simulation_steps = 50;

% Preallocate arrays:
error_lq_disturbance = zeros(3, number_of_simulation_steps, number_of_simulations);
error_lq_no_disturbance = zeros(3, number_of_simulation_steps, number_of_simulations);
setpoint_array_common = zeros(3, number_of_simulation_steps, number_of_simulations);

u_lq_disturbance = zeros(3, number_of_simulation_steps-1, number_of_simulations);
u_lq_no_disturbance = zeros(3, number_of_simulation_steps-1, number_of_simulations);

%%%%%%%%%%%%%%%%%%%%%%
%%% Setpoint error %%%
%%%%%%%%%%%%%%%%%%%%%%

% Read data from files
for iteration=1:number_of_simulations
    
    % Get data from simulation with external disturbance
    sim = load(strcat('Workspace\lq_dp_model_dist_data_', num2str(iteration), '.mat'));
    error_lq_disturbance(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps) - sim.x_array(1:3,1:number_of_simulation_steps); 
    setpoint_array_common(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps);
    u_lq_disturbance(:,:,iteration) = sim.u_array(:,1:number_of_simulation_steps-1);

    % Get data from simulation with no external disturbance
    sim = load(strcat('Workspace\lq_dp_model_no_dist_data_', num2str(iteration), '.mat'));
    error_lq_no_disturbance(:,:,iteration) = sim.setpoint(:,1:number_of_simulation_steps) - sim.x_array(1:3,1:number_of_simulation_steps); 
    u_lq_no_disturbance(:,:,iteration) = sim.u_array(:,1:number_of_simulation_steps-1);
end

% Get RMSE
error_lq_disturbance_rmse = squeeze(sqrt(sum(error_lq_disturbance.^2,2)/number_of_simulation_steps));
error_lq_no_disturbance_rmse = squeeze(sqrt(sum(error_lq_no_disturbance.^2,2)/number_of_simulation_steps));

% Get Integrated Absolute Error (IAE)
error_lq_disturbance_iae = squeeze(sum(abs(error_lq_disturbance),2));
error_lq_no_disturbance_iae = squeeze(sum(abs(error_lq_no_disturbance),2));

% Get Total Value (TV)
tv_lq_disturbance = squeeze(sum(abs(u_lq_disturbance),2));
tv_lq_no_disturbance = squeeze(sum(abs(u_lq_no_disturbance),2));

% Calculate total change in heading
heading_changes = squeeze(setpoint_array_common(3, [1, time_of_heading_changes], :));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot simulation results %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);

dp = get(groot, 'DefaultFigurePosition');   % Default position

f1 = figure('DefaultAxesFontSize', 20, 'Position', [dp(1), 0.3*dp(2), dp(3), 1.3*dp(4)]);
t = tiledlayout(4, 1, "TileSpacing", "compact");

%% RMSE without disturbance
nexttile;
hold on;
plot(error_lq_no_disturbance_rmse(1,:));
plot(error_lq_no_disturbance_rmse(2,:));
plot(error_lq_no_disturbance_rmse(3,:));
grid();
title('RMSE LQ optimal control (no dist)');
xlabel('Simulation number');
ylabel('RMSE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position north', 'Postion east', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

rmse_mean_no_disturbance = mean(error_lq_no_disturbance_rmse, 2);
disp("RMSE value for LQ optimal control simulations without disturbance: ");
disp(rmse_mean_no_disturbance);
disp("Total:");
disp(sum(rmse_mean_no_disturbance));

%% RMSE with disturbance
nexttile;
hold on;
plot(error_lq_disturbance_rmse(1,:));
plot(error_lq_disturbance_rmse(2,:));
plot(error_lq_disturbance_rmse(3,:));
grid();
title('RMSE LQ optimal control (inc. dist)');
xlabel('Simulation number');
ylabel('RMSE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position north', 'Postion east', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

rmse_mean_with_disturbance = mean(error_lq_disturbance_rmse, 2);
disp("RMSE value for LQ optimal control simulations with disturbance: ");
disp(rmse_mean_with_disturbance);
disp("Total:");
disp(sum(rmse_mean_with_disturbance));

%% IAE without disturbance
nexttile;
hold on;
plot(error_lq_no_disturbance_iae(1,:));
plot(error_lq_no_disturbance_iae(2,:));
plot(error_lq_no_disturbance_iae(3,:));
grid();
title('IAE LQ optimal control (no dist)');
xlabel('Simulation number');
ylabel('IAE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position north', 'Postion east', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

iae_mean_no_disturbance = mean(error_lq_no_disturbance_iae, 2);
disp("IAE value for LQ optimal control simulations without disturbance: ");
disp(iae_mean_no_disturbance);
disp("Total:");
disp(sum(iae_mean_no_disturbance));

%% IAE with disturbance
nexttile;
hold on;
plot(error_lq_disturbance_iae(1,:));
plot(error_lq_disturbance_iae(2,:));
plot(error_lq_disturbance_iae(3,:));
grid();
title('IAE LQ optimal control (inc. dist)');
xlabel('Simulation number');
ylabel('IAE');
% legend({'Position North', 'Postion East', 'Yaw'}, 'Location', 'Best');
legend({'Position north', 'Postion east', 'Yaw'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

iae_mean_with_disturbance = mean(error_lq_disturbance_iae, 2);
disp("IAE value for LQ optimal control simulations with disturbance: ");
disp(iae_mean_with_disturbance);
disp("Total:");
disp(sum(iae_mean_with_disturbance));

save_plot(f1, 'mc_results', 'Results\');

disp("Diff RMSE: ");
diff_rmse = rmse_mean_with_disturbance - rmse_mean_no_disturbance;
disp(diff_rmse);

disp("Diff IAE: ");
diff_iae = iae_mean_with_disturbance - iae_mean_no_disturbance;
disp(diff_iae);

tv_mean_no_disturbance = mean(tv_lq_no_disturbance, 2);
disp("TV value for LQ optimal control simulations without disturbance: ");
disp(tv_mean_no_disturbance);
disp("Total:");
disp(sum(tv_mean_no_disturbance));

tv_mean_disturbance = mean(tv_lq_disturbance, 2);
disp("TV value for LQ optimal control simulations with disturbance: ");
disp(tv_mean_disturbance);
disp("Total:");
disp(sum(tv_mean_disturbance));

% Display total heading change
total_heading_change_array = zeros(1,number_of_simulations);
for i=1:number_of_simulations
    total_heading_change_array(1,i) = sum(get_smallest_angle_differences(heading_changes(:,i)));
end

show_angle_changes = true;
if (show_angle_changes)
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
end

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
