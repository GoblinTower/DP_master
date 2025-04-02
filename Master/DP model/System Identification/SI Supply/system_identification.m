clear, clc, close all;

addpath("..\..\..\..\Libraries\DSR");

% Fetch logged data from file
data = load("Log\dsr_supply_data");

% x_array
% x[1:3]: x, y, psi (position in north, east and vessel heading)
% x[4:6]: u, v, r (velocity in surge, sway and yaw)
% We want to identify the surge, sway and yaw rotation dynamics
y_array = [data.x_array(4:6,:)];
u_array = [data.u_array(1:3,:)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System identification - DSR %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System identification parameters (DSR)
L_sys = 3;              % Number of block rows in extended observability matrix
n_sys = 3;              % Number of states
g_sys = 0;              % Structure parameter
M_sys = 1;

[A,B,C,E,CF,F,x0] = dsr(y_array',u_array',L_sys,g_sys,L_sys,M_sys,n_sys);

save ..\Log\ssm_dsr_supply A B C;
