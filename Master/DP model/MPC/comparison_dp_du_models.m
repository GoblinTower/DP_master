% Simple script for plotting and comparing MPC control simulations
% This covers simulations running using delta u formulation. 
% This ensures convergence to setpoint over time.
clear, clc, close all;

% Load data
% Files with disturbance
dp_psi = load('Workspace\mpc_const_psi_du_with_dist.mat');
dp_r = load('Workspace\mpc_const_r_du_with_dist.mat');
dp_lpv = load('Workspace\mpc_lpv_du_with_dist.mat');

% Files without disturbance
% dp_psi = load('Workspace\mpc_const_psi_du_without_dist.mat');
% dp_r = load('Workspace\mpc_const_r_du_without_dist.mat');
% dp_lpv = load('Workspace\mpc_lpv_du_without_dist.mat');

%%%%%%%%%%%%%%%%%%
%%% Path plots %%%
%%%%%%%%%%%%%%%%%%
f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, 3, "TileSpacing", "compact");

nexttile;
hold on;
plot(dp_psi.x_array(2,:), dp_psi.x_array(1,:))
plot(dp_psi.x_array(2,1), dp_psi.x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
plot(dp_psi.x_array(2,end), dp_psi.x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
grid();
title('Path (constant \psi)');
xlabel('East [m]');
ylabel('North [m]');
xlim([-15,7]);
ylim([-12,12]);
legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(dp_r.x_array(2,:), dp_r.x_array(1,:), 'LineWidth', 1.25)
plot(dp_r.x_array(2,1), dp_r.x_array(1,1), 'ko', 'MarkerSize', 15, 'LineWidth', 3);      % Start position
plot(dp_r.x_array(2,end), dp_r.x_array(1,end), 'ro', 'MarkerSize', 15, 'LineWidth', 3);  % End position
grid();
title('Path (constant r)');
xlabel('East [m]');
ylabel('North [m]');
xlim([-15,7]);
ylim([-12,12]);
legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(dp_lpv.x_array(2,:), dp_lpv.x_array(1,:), 'LineWidth', 1.25)
plot(dp_lpv.x_array(2,1), dp_lpv.x_array(1,1), 'ko', 'MarkerSize', 15, 'LineWidth', 3);      % Start position
plot(dp_lpv.x_array(2,end), dp_lpv.x_array(1,end), 'ro', 'MarkerSize', 15, 'LineWidth', 3);  % End position
grid();
title('Path (LPV)');
xlabel('East [m]');
ylabel('North [m]');
xlim([-15,7]);
ylim([-12,12]);
legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

saveas(f1, 'Workspace\Plots\path.png');

%%%%%%%%%%%%%%
%%% Forces %%%
%%%%%%%%%%%%%%
f2 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 3, "TileSpacing", "compact");

length_time = min(size(dp_psi.t_array,2), size(dp_psi.u_array,2));

% Surge (constant psi)
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.u_array(1,:))
grid();
title('Force in surge (constant \psi)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Surge (constant r)
nexttile;
hold on;
plot(dp_r.t_array(1:length_time), dp_r.u_array(1,:))
grid();
title('Force in surge (constant r)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Surge (LPV)
nexttile;
hold on;
plot(dp_lpv.t_array(1:length_time), dp_lpv.u_array(1,:))
grid();
title('Force in surge (LPV)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Sway (constant psi)
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.u_array(2,:))
grid();
title('Force in sway (constant \psi)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Sway (constant r)
nexttile;
hold on;
plot(dp_r.t_array(1:length_time), dp_r.u_array(2,:))
grid();
title('Force in sway (constant r)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Sway (LPV)
nexttile;
hold on;
plot(dp_lpv.t_array(1:length_time), dp_lpv.u_array(2,:))
grid();
title('Force in sway (constant r)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Yaw (constant psi)
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.u_array(3,:))
grid();
title('Momentum in yaw (constant \psi)');
xlabel('t [s]');
ylabel('Momentum [Nm]');
grid on, grid minor;
box on;
hold off;

% Yaw (constant r)
nexttile;
hold on;
plot(dp_r.t_array(1:length_time), dp_r.u_array(3,:))
grid();
title('Momentum in yaw (constant r)');
xlabel('t [s]');
ylabel('Momentum [Nm]');
grid on, grid minor;
box on;
hold off;

% Yaw (LPV)
nexttile;
hold on;
plot(dp_lpv.t_array(1:length_time), dp_lpv.u_array(3,:))
grid();
title('Momentum in yaw (LPV)');
xlabel('t [s]');
ylabel('Momentum [Nm]');
grid on, grid minor;
box on;
hold off;

saveas(f2, 'Workspace\Plots\forces.png');

%%%%%%%%%%%%%%%%%%%%
%%% Position NED %%%
%%%%%%%%%%%%%%%%%%%%
f3 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 3, "TileSpacing", "compact");

% Position north (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), dp_psi.x_array(1,1:length_time));
plot(dp_psi.t_array(1:length_time), dp_psi.setpoint(1,1:length_time));
grid();
title('Position north (constant \psi)');
xlabel('t [s]');
ylabel('North, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position north (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), dp_r.x_array(1,1:length_time));
plot(dp_r.t_array(1:length_time), dp_r.setpoint(1,1:length_time));
grid();
title('Position north (constant r)');
xlabel('t [s]');
ylabel('North, y [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position north (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), dp_lpv.x_array(1,1:length_time));
plot(dp_lpv.t_array(1:length_time), dp_lpv.setpoint(1,1:length_time));
grid();
title('Position north (LPV)');
xlabel('t [s]');
ylabel('North, y [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position east (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), dp_psi.x_array(2,1:length_time));
plot(dp_psi.t_array(1:length_time), dp_psi.setpoint(2,1:length_time));
grid();
title('Position east (constant \psi)');
xlabel('t [s]');
ylabel('East, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position east (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), dp_r.x_array(2,1:length_time));
plot(dp_r.t_array(1:length_time), dp_r.setpoint(2,1:length_time));
grid();
title('Position east (constant r)');
xlabel('t [s]');
ylabel('East, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position east (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), dp_lpv.x_array(2,1:length_time));
plot(dp_lpv.t_array(1:length_time), dp_lpv.setpoint(2,1:length_time));
grid();
title('Position east (LPV)');
xlabel('t [s]');
ylabel('East, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Heading (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), rad2deg(dp_psi.x_array(3,1:length_time)));
plot(dp_psi.t_array(1:length_time), rad2deg(dp_psi.setpoint(3,1:length_time)));
grid();
title('Heading (constant \psi)');
xlabel('t [s]');
ylabel('Heading, \psi [°]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Heading (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), rad2deg(dp_r.x_array(3,1:length_time)));
plot(dp_r.t_array(1:length_time), rad2deg(dp_r.setpoint(3,1:length_time)));
grid();
title('Heading (constant r)');
xlabel('t [s]');
ylabel('Heading, \psi [°]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Heading (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), rad2deg(dp_lpv.x_array(3,1:length_time)));
plot(dp_lpv.t_array(1:length_time), rad2deg(dp_lpv.setpoint(3,1:length_time)));
grid();
title('Heading (LPV)');
xlabel('t [s]');
ylabel('Heading, \psi [°]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

saveas(f3, 'Workspace\Plots\setpoint_tracking.png');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity and angular momentum in BODY frame %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f4 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 3, "TileSpacing", "compact");

% Velocity surge (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), dp_psi.x_array(4,1:length_time));
grid();
title('Velocity surge (constant \psi)');
xlabel('t [s]');
ylabel('Velocity, u [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity surge (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), dp_r.x_array(4,1:length_time));
grid();
title('Velocity surge (constant r)');
xlabel('t [s]');
ylabel('Velocity, u [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity surge (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), dp_lpv.x_array(4,1:length_time));
grid();
title('Velocity surge (LPV)');
xlabel('t [s]');
ylabel('Velocity, u [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), dp_psi.x_array(5,1:length_time));
grid();
title('Velocity sway (constant \psi)');
xlabel('t [s]');
ylabel('Velocity, v [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), dp_r.x_array(5,1:length_time));
grid();
title('Velocity sway (constant r)');
xlabel('t [s]');
ylabel('Velocity, v [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), dp_lpv.x_array(5,1:length_time));
grid();
title('Velocity sway (LPV)');
xlabel('t [s]');
ylabel('Velocity, v [m/s]');
grid on, grid minor;
box on;
hold off;

% Rotation yaw (constant psi)
nexttile;
hold on
plot(dp_psi.t_array(1:length_time), dp_psi.x_array(6,1:length_time));
grid();
title('Rotation in yaw (constant \psi)');
xlabel('t [s]');
ylabel('Angular rotation, r [rad/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (constant r)
nexttile;
hold on
plot(dp_r.t_array(1:length_time), dp_r.x_array(6,1:length_time));
grid();
title('Rotation in yaw (constant r)');
xlabel('t [s]');
ylabel('Angular rotation, r [rad/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (LPV)
nexttile;
hold on
plot(dp_lpv.t_array(1:length_time), dp_lpv.x_array(6,1:length_time));
grid();
title('Rotation in yaw (LPV)');
xlabel('t [s]');
ylabel('Angular rotation, r [rad/s]');
grid on, grid minor;
box on;
hold off;

saveas(f4, 'Workspace\Plots\velocity.png');

%%%%%%%%%%%%%%%%%%%
%%% Kalman gain %%%
%%%%%%%%%%%%%%%%%%%
f5 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, 3, "TileSpacing", "compact");

nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.K_array(:,1:length_time));
grid();
title('Kalman gain values (constant \psi)');
xlabel('t [s]');
ylabel('Gain matrix entries');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(dp_r.t_array(1:length_time), dp_r.K_array(:,1:length_time));
grid();
title('Kalman gain values (constant r)');
xlabel('t [s]');
ylabel('Gain matrix entries');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(dp_lpv.t_array(1:length_time), dp_lpv.K_array(:,1:length_time));
grid();
title('Kalman gain values (LPV)');
xlabel('t [s]');
ylabel('Gain matrix entries');
grid on, grid minor;
box on;
hold off;

saveas(f5, 'Workspace\Plots\kalman.png');

%%%%%%%%%%%%
%%% Wind %%%
%%%%%%%%%%%%
f6 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(5, 1, "TileSpacing", "compact");

% Absolute velocity
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.wind_abs(1:length_time));
grid();
title('Wind');
xlabel('t [s]');
ylabel('Velocity [m/s]');
grid on, grid minor;
box on;
hold off;

% Angle of attack
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), rad2deg(dp_psi.wind_beta(1:length_time)));
grid();
title('Wind angle');
xlabel('t [s]');
ylabel('Degree [°]');
grid on, grid minor;
box on;
hold off;

% Force in surge
nexttile;
plot(dp_psi.t_array(1:length_time), dp_psi.wind_force_array(1,1:length_time));
grid();
title('Force in surge');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Force in sway
nexttile;
plot(dp_psi.t_array(1:length_time), dp_psi.wind_force_array(2,1:length_time));
grid();
title('Force in sway');
xlabel('t [s]');
ylabel('F [N]');
grid on, grid minor;
box on;
hold off;

% Momentum in yaw
nexttile;
plot(dp_psi.t_array(1:length_time), dp_psi.wind_force_array(3,1:length_time));
grid();
title('Momentum in yaw');
xlabel('t [s]');
ylabel('N [Nm]');
grid on, grid minor;
box on;
hold off;

saveas(f6, 'Workspace\Plots\wind.png');

%%%%%%%%%%%%%%%
%%% Current %%%
%%%%%%%%%%%%%%%
f7 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, 2, "TileSpacing", "compact");

% Current in north direction
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.current_force(1,1:length_time));
plot(dp_psi.t_array(1:length_time), dp_psi.x_est_array(7,1:length_time));
grid();
title('Current in north direction');
xlabel('t [s]');
ylabel('Force in North [N]');
legend({'Current force', 'Est. current force'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Current in east direction
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.current_force(2,1:length_time));
plot(dp_psi.t_array(1:length_time), dp_psi.x_est_array(8,1:length_time));
grid();
title('Current in east direction');
xlabel('t [s]');
ylabel('Force in east [N]');
legend({'Current force', 'Est. current force'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

saveas(f7, 'Workspace\Plots\current.png');

%%%%%%%%%%%%%
%%% Waves %%%
%%%%%%%%%%%%%
f8 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 1, "TileSpacing", "compact");

% Wave force in north direction
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.wave_force(1,1:length_time))
grid();
title('Waves in north direction');
xlabel('t [s]');
ylabel('Force in north [N]');
grid on, grid minor;
box on;
hold off;

% Wave force in east direction
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.wave_force(2,1:length_time))
grid();
title('Waves in east direction');
xlabel('t [s]');
ylabel('Force in east [N]');
grid on, grid minor;
box on;
hold off;

% Wave momentum in yaw
nexttile;
hold on;
plot(dp_psi.t_array(1:length_time), dp_psi.wave_force(3,1:length_time))
grid();
title('Wave momentum in yaw');
xlabel('t [s]');
ylabel('Force in yaw [Nm]');
grid on, grid minor;
box on;
hold off;

saveas(f8, 'Workspace\Plots\waves.png');

%%%%%%%%%%%
%%% IAE %%%
%%%%%%%%%%%

% Constant_psi
error_psi = abs(dp_psi.setpoint(:,1:length_time) - dp_psi.x_array(1:3,1:length_time));

% Constant error
error_r = abs(dp_r.setpoint(:,1:length_time) - dp_r.x_array(1:3,1:length_time));

% LPV
error_lpv = abs(dp_lpv.setpoint(:,1:length_time) - dp_lpv.x_array(1:3,1:length_time));

disp("Integrated Absolute Error (IAE):");

% IAE
iae_psi = sum(error_psi,2)
iae_r = sum(error_r,2)
iae_lpv = sum(error_lpv,2)

%%%%%%%%%%
%%% TV %%%
%%%%%%%%%%

disp("Total Value (TV):");

% Constant_psi
u_total_value_psi = sum(abs(dp_psi.u_array(:,1:length_time)),2)

% Constant error
u_total_value_r = sum(abs(dp_r.u_array(:,1:length_time)),2)

% LPV
u_total_value_lpv = sum(abs(dp_lpv.u_array(:,1:length_time)),2)
