% Simple script for plotting time difference between LQ model that utilize
% prestored matrices and non-prestored matrices (calculated continuously).
clear, clc, close all;

time_data = load('time_measured.mat');
 
% Plot time
f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
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

% Calculate metrics
mean_stored = mean(time_data.time_array_stored_matrices);
disp(strcat(['Mean simulation time for stored matrices simulation: ', num2str(mean_stored)]));

mean_non_stored = mean(time_data.time_array_continuously_calculated);
disp(strcat(['Mean simulation time for non-stored matrices simulation: ', num2str(mean_non_stored)]));
