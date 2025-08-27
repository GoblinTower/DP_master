% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

lq_stored = load('Workspace\lq_dp_model_stored_matrices_dist_data.mat');
lq_normal = load('Workspace\lq_dp_model_no_stored_matrices_dist_data.mat');
 
lmf = [lq_stored, lq_normal];                               % Shortcut for List of .Mat Files
nos = {'LQ control stored', 'LQ control normal'};           % Shortcut for Name Of Simulations

storage_path = 'Workspace\TimeComparison';                  % Path where plots are stored

show_setpoints = true;                                      % Show setpoints in plot

[iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints);
