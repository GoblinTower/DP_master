clear, clc, close all;

addpath("..\..\..\..\Libraries\DSR");

% Fetch logged data from file
data = load("Log\dsr_osv_data.mat");

% x_array
% x[1:3]: u, v, w (velocity in surge, sway and heave)
% x[4:6]: p, q, r (angular velocity in roll, pitch and yaw)
% x[7:9]: x, y, z (position, relative to start position)
% x[10:12]: phi, theta, psi: (angular position in roll, pitch and yaw)
% Assume position (GPS) and velocity is known
y_array = [data.x_array(1:2,:); data.x_array(6,:)];
u_array = [data.u_squared(1:4,:)]; % degree2rad(azimuth_angle_1),deg2rad(azimuth_angle_2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System identification - DSR %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System identification parameters (DSR)
L_sys = 3;              % Number of block rows in extended observability matrix
n_sys = 6;              % Number of states
g_sys = 0;              % Structure parameter
M_sys = 1;

[A,B,C,E,CF,F,x0] = dsr(y_array',u_array',L_sys,g_sys,L_sys,M_sys,n_sys);

save ..\Log\ssm_dsr_osv A B C;
