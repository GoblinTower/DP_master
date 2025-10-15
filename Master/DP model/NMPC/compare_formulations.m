% Simple script for plotting results from NMPC. Compares the model that
% uses du vs u in the objective function.
clear, clc, close all;

% Load data
% Files with disturbance
% dp_u = load('Workspace\nmpc_du_dist_data.mat');
dp_du = load('Workspace\nmpc_du_no_dist_data.mat');
dp_u = load('Workspace\nmpc_du_no_dist_data.mat');

% Files without disturbance
% dp_u = load('Workspace\nmpc_dist_data.mat');
% dp_du = load('Workspace\nmpc_no_dist_data.mat');

%%%%%%%%%%%%%%%%%%
%%% Path plots %%%
%%%%%%%%%%%%%%%%%%
f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, 2, "TileSpacing", "compact");

nexttile;
hold on;
plot(dp_du.x_array(2,:), dp_du.x_array(1,:))
plot(dp_du.x_array(2,1), dp_du.x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
plot(dp_du.x_array(2,end), dp_du.x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
grid();
title('Path (\Deltau formulation)');
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
plot(dp_u.x_array(2,:), dp_u.x_array(1,:), 'LineWidth', 1.25)
plot(dp_u.x_array(2,1), dp_u.x_array(1,1), 'ko', 'MarkerSize', 15, 'LineWidth', 3);      % Start position
plot(dp_u.x_array(2,end), dp_u.x_array(1,end), 'ro', 'MarkerSize', 15, 'LineWidth', 3);  % End position
grid();
title('Path (u formulation)');
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
t = tiledlayout(3, 2, "TileSpacing", "compact");

length_time = min(size(dp_du.t_array,2), size(dp_du.u_array,2));

% Surge (du formulation)
nexttile;
hold on;
plot(dp_du.t_array(1:length_time), dp_du.u_array(1,:))
grid();
title('Force in surge (\Deltau formulation)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Surge (u formulation)
nexttile;
hold on;
plot(dp_u.t_array(1:length_time), dp_u.u_array(1,:))
grid();
title('Force in surge (u formulation)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Sway (du formulation)
nexttile;
hold on;
plot(dp_du.t_array(1:length_time), dp_du.u_array(2,:))
grid();
title('Force in sway (\Deltau formulation)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Sway (u formulation)
nexttile;
hold on;
plot(dp_u.t_array(1:length_time), dp_u.u_array(2,:))
grid();
title('Force in sway (u formulation)');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Yaw (du formulation)
nexttile;
hold on;
plot(dp_du.t_array(1:length_time), dp_du.u_array(3,:))
grid();
title('Momentum in yaw (\Deltau formulation)');
xlabel('t [s]');
ylabel('Momentum [Nm]');
grid on, grid minor;
box on;
hold off;

% Yaw (u formulation)
nexttile;
hold on;
plot(dp_u.t_array(1:length_time), dp_u.u_array(3,:))
grid();
title('Momentum in yaw (u formulation)');
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
t = tiledlayout(3, 2, "TileSpacing", "compact");

% Position north (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), dp_du.x_array(1,1:length_time));
plot(dp_du.t_array(1:length_time), dp_du.setpoint(1,1:length_time));
grid();
title('Position north (\Deltau formulation)');
xlabel('t [s]');
ylabel('North, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position north (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), dp_u.x_array(1,1:length_time));
plot(dp_u.t_array(1:length_time), dp_u.setpoint(1,1:length_time));
grid();
title('Position north (u formulation)');
xlabel('t [s]');
ylabel('North, y [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position east (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), dp_du.x_array(2,1:length_time));
plot(dp_du.t_array(1:length_time), dp_du.setpoint(2,1:length_time));
grid();
title('Position east (\Deltau formulation)');
xlabel('t [s]');
ylabel('East, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Position east (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), dp_u.x_array(2,1:length_time));
plot(dp_u.t_array(1:length_time), dp_u.setpoint(2,1:length_time));
grid();
title('Position east (u formulation)');
xlabel('t [s]');
ylabel('East, x [m]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Heading (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), rad2deg(dp_du.x_array(3,1:length_time)));
plot(dp_du.t_array(1:length_time), rad2deg(dp_du.setpoint(3,1:length_time)));
grid();
title('Heading (\Deltau formulation)');
xlabel('t [s]');
ylabel('Heading, \psi [°]');
legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

% Heading (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), rad2deg(dp_u.x_array(3,1:length_time)));
plot(dp_u.t_array(1:length_time), rad2deg(dp_u.setpoint(3,1:length_time)));
grid();
title('Heading (u formulation)');
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
t = tiledlayout(3, 2, "TileSpacing", "compact");

% Velocity surge (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), dp_du.x_array(4,1:length_time));
grid();
title('Velocity surge (\Deltau formulation)');
xlabel('t [s]');
ylabel('Velocity, u [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity surge (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), dp_u.x_array(4,1:length_time));
grid();
title('Velocity surge (u formulation)');
xlabel('t [s]');
ylabel('Velocity, u [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), dp_du.x_array(5,1:length_time));
grid();
title('Velocity sway (\Deltau formulation)');
xlabel('t [s]');
ylabel('Velocity, v [m/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), dp_u.x_array(5,1:length_time));
grid();
title('Velocity sway (u formulation)');
xlabel('t [s]');
ylabel('Velocity, v [m/s]');
grid on, grid minor;
box on;
hold off;

% Rotation yaw (du formulation)
nexttile;
hold on
plot(dp_du.t_array(1:length_time), dp_du.x_array(6,1:length_time));
grid();
title('Rotation in yaw (\Deltau formulation)');
xlabel('t [s]');
ylabel('Angular rotation, r [rad/s]');
grid on, grid minor;
box on;
hold off;

% Velocity sway (u formulation)
nexttile;
hold on
plot(dp_u.t_array(1:length_time), dp_u.x_array(6,1:length_time));
grid();
title('Rotation in yaw (u formulation)');
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
t = tiledlayout(1, 2, "TileSpacing", "compact");

nexttile;
hold on;
plot(dp_du.t_array(1:length_time), dp_du.K_array(:,1:length_time));
grid();
title('Kalman gain values (\Deltau formulation)');
xlabel('t [s]');
ylabel('Gain matrix entries');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(dp_u.t_array(1:length_time), dp_u.K_array(:,1:length_time));
grid();
title('Kalman gain values (u formulation)');
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
plot(dp_du.t_array(1:length_time), dp_du.wind_abs(1:length_time));
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
plot(dp_du.t_array(1:length_time), rad2deg(dp_du.wind_beta(1:length_time)));
grid();
title('Wind angle');
xlabel('t [s]');
ylabel('Degree [°]');
grid on, grid minor;
box on;
hold off;

% Force in surge
nexttile;
plot(dp_du.t_array(1:length_time), dp_du.wind_force_array(1,1:length_time));
grid();
title('Force in surge');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Force in sway
nexttile;
plot(dp_du.t_array(1:length_time), dp_du.wind_force_array(2,1:length_time));
grid();
title('Force in sway');
xlabel('t [s]');
ylabel('F [N]');
grid on, grid minor;
box on;
hold off;

% Momentum in yaw
nexttile;
plot(dp_du.t_array(1:length_time), dp_du.wind_force_array(3,1:length_time));
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
plot(dp_du.t_array(1:length_time), dp_du.current_force(1,1:length_time));
plot(dp_du.t_array(1:length_time), dp_du.x_est_array(7,1:length_time));
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
plot(dp_du.t_array(1:length_time), dp_du.current_force(2,1:length_time));
plot(dp_du.t_array(1:length_time), dp_du.x_est_array(8,1:length_time));
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
plot(dp_du.t_array(1:length_time), dp_du.wave_force(1,1:length_time))
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
plot(dp_du.t_array(1:length_time), dp_du.wave_force(2,1:length_time))
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
plot(dp_du.t_array(1:length_time), dp_du.wave_force(3,1:length_time))
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
error_du = abs(dp_du.setpoint(:,1:length_time) - dp_du.x_array(1:3,1:length_time));

% Constant error
error_u = abs(dp_u.setpoint(:,1:length_time) - dp_u.x_array(1:3,1:length_time));

disp("Integrated Absolute Error (IAE):");

% IAE
iae_du = sum(error_du,2)
iae_u = sum(error_u,2)

%%%%%%%%%%
%%% TV %%%
%%%%%%%%%%

disp("Total Value (TV):");

% Constant_psi
u_total_value_du = sum(abs(dp_du.u_array(:,1:length_time)),2)

% Constant error
u_total_value_u = sum(abs(dp_u.u_array(:,1:length_time)),2)
