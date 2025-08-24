% Simple script for testing the prbs1() function.
% PRBS: Pseudo Random Binary Signal.
% This function was developed by David Di Ruscio.
clear, clc, close all;

addpath("..\");

Tmin = 50;      % Maximum interval of constant signal
Tmax = 150;     % Minimum interval of constant signal
N = 2000;       % Number of samples
t = 1:1:2000;   % Time of samples

[u, ~] = prbs1(N, Tmin, Tmax);

% Plot PRBS signal
fig = figure(1);
plot(t, u);
grid();
title('Pseudo Random Binary Signal (PRBS)');
ylim([-1.2, 1.2]);
xlabel('t [time]');
ylabel('u [signal]');

% Store plot
save_plot(fig, "PRBS_example", "Results/PRBS");
