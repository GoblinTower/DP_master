% Script for performing system identification of Balchen model data
addpath("..\..\..\..\Libraries\DSR");

% Fetch logged data from file
data = load("Log\dsr_balchen_data");

% x_array
% x(1)               : North position (x)
% x(2)               : East position (y)
% x(3)               : Vessel heading relative to north (psi)
% x(4)               : Distance travelled in surge direction (x_su)
% x(5)               : Distance travelled in sway direction (y_su)
% x(6)               : Velocity in surge (u)
% x(7)               : Velocity in sway (v)
% x(8)               : Vessel heading angular velocity (r)
% We want to identify the surge, sway and yaw rotation dynamics
y_array = [data.x_array(6:8,:)];
u_array = [data.u_array(1:3,:)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System identification - DSR %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System identification parameters (DSR)
L_sys = 3;              % Number of block rows in extended observability matrix
n_sys = n_si_dim;       % Number of states
g_sys = 0;              % Structure parameter
M_sys = 1;

[A,B,C,E,CF,F,x0] = dsr(y_array',u_array',L_sys,g_sys,L_sys,M_sys,n_sys);

save ..\Log\ssm_dsr_balchen dt A B C;
