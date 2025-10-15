% Simple script for plotting time difference between LQ model that utilize
% prestored matrices and non-prestored matrices (calculated continuously).
clear, clc, close all;

time_data = load('time_measured.mat');
 
% Plot time
% f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
f1 = figure('DefaultAxesFontSize', 20);
t = tiledlayout(1, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(time_data.time_array_stored_matrices, 'r*');
plot(time_data.time_array_continuously_calculated, 'k*');
grid();
title('Simulation time for stored versus continuously calculated matrices');
xlabel('Simulation number');
ylabel('Time [s]');
legend({'Stored matrices simulation', 'Non-stored matrices simulation'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

save_plot(f1, 'time', 'plots');

% Calculate metrics
mean_stored = mean(time_data.time_array_stored_matrices);
disp(strcat(['Mean simulation time for stored matrices simulation: ', num2str(mean_stored)]));

mean_non_stored = mean(time_data.time_array_continuously_calculated);
disp(strcat(['Mean simulation time for non-stored matrices simulation: ', num2str(mean_non_stored)]));

% Time reduction
reduced_time = mean_stored/mean_non_stored;
disp(strcat(['Reduced time in percent: ', num2str(reduced_time*100)]));

faster = mean_non_stored/mean_stored;
disp(strcat(['Prestored matrices mode runs: ', num2str(faster), 'faster than non-prestored matrices mode']));

time_diff = mean_non_stored - mean_stored;
disp(strcat(['Elapsed time difference: ', num2str(time_diff)]));

% Difference in IAE

% n_simulations = size(time_data.time_array_stored_matrices,2);
% 
% % Preallocate arrays
% error_stored = zeros(3, n_simulations);
% error_non_stored = zeros(3, n_simulations);
% iae_stored = zeros(3, n_simulations);
% iae_non_stored = zeros(3 ,n_simulations);
% 
% for i=1:n_simulations
% 
%     % Get stored matrices data
%     pre = load(strcat('..\Workspace\Prestored\prestored_matrices_', num2str(i), '.mat'));
%     len = size(pre.x_array,2);
% 
%     % Calculate IAE stored matrices
%     iae_stored(:,i) = sum(abs(pre.setpoint(:,1:len) - pre.x_array(1:3,1:len)),2);
% 
%     con = load(strcat('..\Workspace\Prestored\continuously_calculated_matrices_', num2str(i), '.mat'));
%     len = size(con.x_array,2);
% 
%     % Calculate IAE non-stored matrices
%     iae_non_stored(:,i) = sum(abs(con.setpoint(:,1:len) - con.x_array(1:3,1:len)),2);
% 
% end
% 
% nexttile;
% hold on;
% plot(iae_stored(1,:), 'r*');
% plot(iae_non_stored(1,:), 'k*');
% grid();
% title('IAE for surge setpoint');
% xlabel('Simulation number');
% ylabel('Time [s]');
% legend({'Stored matrices simulation', 'Non-stored matrices simulation'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(iae_stored(2,:), 'r*');
% plot(iae_non_stored(2,:), 'k*');
% grid();
% title('IAE for sway setpoint');
% xlabel('Simulation number');
% ylabel('Time [s]');
% legend({'Stored matrices simulation', 'Non-stored matrices simulation'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(iae_stored(3,:), 'r*');
% plot(iae_non_stored(3,:), 'k*');
% grid();
% title('IAE for yaw setpoint');
% xlabel('Simulation number');
% ylabel('Time [s]');
% legend({'Stored matrices simulation', 'Non-stored matrices simulation'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
