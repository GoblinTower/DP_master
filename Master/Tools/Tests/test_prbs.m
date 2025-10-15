% Simple script for testing the prbs1() function.
% PRBS: Pseudo Random Binary Signal.
% This function was developed by David Di Ruscio.
clear, clc, close all;
%% 

addpath("..\");

Tmin = 10;      % Maximum interval of constant signal
Tmax = 50;      % Minimum interval of constant signal
N = 2000;       % Number of samples
t = 1:1:2000;   % Time of samples

[u, ~] = prbs1(N, Tmin, Tmax);

% Plot PRBS signal
fig = figure('DefaultAxesFontSize', 20);
tl = tiledlayout(1, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(t, u);
grid();
title('PRBS example');
ylim([-1.2, 1.2]);
xlabel('t [sample]');
ylabel('u [signal]');
grid on, grid minor;
box on;
hold off;

% Store plot
save_plot(fig, "PRBS_example", "Results/PRBS");
