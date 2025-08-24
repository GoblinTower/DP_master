% Run all simulations
% intended for saving time
clear, clc, close all;

% u-formulation
run 'mpc_constant_psi';
run 'mpc_constant_r';
run 'mpc_lpv';

% du-formulation
run 'mpc_delta_u_constant_psi';
run 'mpc_delta_u_constant_r';
run 'mpc_delta_u_lpv';
run 'mpc_delta_u_recalculating_setpoint';