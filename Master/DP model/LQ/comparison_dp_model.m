% Simple script for plotting and comparing LQ control simulations
% In this case two LQ optimal control simulations are performed and
% compared. They are run with and without external disturbances
% respectively.
clear, clc, close all;

% Load data
dp_no_dist = load('Workspace\lq_dp_model_no_dist_data.mat');
dp_dist = load('Workspace\lq_dp_model_dist_data.mat');

lmf = [dp_no_dist, dp_dist];                                % Shortcut for List of .Mat Files
nos = {'no dist', 'inc. dist'};                             % Shortcut for Name Of Simulations

storage_folder = 'Workspace\DistComparison';                % Path where plots are stored

show_setpoints = true;                                      % Show setpoints in plot

plot_details.path_xlim = [-7,7];
plot_details.path_ylim = [-7,12];

plot_details.force_surge_exponent = 5;
plot_details.force_sway_exponent = 5;
plot_details.momentum_yaw_exponent = 6;

plot_details.kalman_exponent = 5;

plot_details.wind_surge_exponent = 4;
plot_details.wind_sway_exponent = 4;
plot_details.wind_momentum_exponent = 5;

plot_details.current_north_exponent = 5;
plot_details.current_east_exponent = 5;

plot_details.wave_north_exponent = 4;
plot_details.wave_east_exponent = 4;
plot_details.wave_momentum_exponent = 3;

show_current_estimate = true;
model_number_environmental = 2;

[iae, tv] = plot_data_extended(lmf, nos, show_setpoints, 20, storage_folder, plot_details, ...
    show_current_estimate, model_number_environmental);

% %%%%%%%%%%%%%%%%%%
% %%% Path plots %%%
% %%%%%%%%%%%%%%%%%%
% % f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f1 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(1, 2, "TileSpacing", "compact");
% 
% nexttile;
% hold on;
% plot(dp_no_dist.x_array(2,:), dp_no_dist.x_array(1,:))
% plot(dp_no_dist.x_array(2,1), dp_no_dist.x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
% plot(dp_no_dist.x_array(2,end), dp_no_dist.x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
% grid();
% title('Path (no disturbance)');
% xlabel('East [m]');
% ylabel('North [m]');
% xlim([-7,7]);
% ylim([-7,12]);
% legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(dp_dist.x_array(2,:), dp_dist.x_array(1,:), 'LineWidth', 1.25)
% plot(dp_dist.x_array(2,1), dp_dist.x_array(1,1), 'ko', 'MarkerSize', 15, 'LineWidth', 3);      % Start position
% plot(dp_dist.x_array(2,end), dp_dist.x_array(1,end), 'ro', 'MarkerSize', 15, 'LineWidth', 3);  % End position
% grid();
% title('Path (including disturbance)');
% xlabel('East [m]');
% ylabel('North [m]');
% xlim([-7,7]);
% ylim([-7,12]);
% legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f1, 'Workspace\Plots\path.png');
% 
% %%%%%%%%%%%%%%
% %%% Forces %%%
% %%%%%%%%%%%%%%
% % f2 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f2 = figure('DefaultAxesFontSize', 18, 'Position', [0,0, 1200, 1600]);
% t = tiledlayout(3, 2, "TileSpacing", "compact");
% 
% length_time = min(size(dp_no_dist.t_array,2), size(dp_no_dist.u_array,2));
% 
% % Surge (no disturbance)
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.u_array(1,:))
% grid();
% title('Force in surge (no disturbance)');
% xlabel('t [s]');
% ylabel('Force [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Surge (including disturbance)
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.u_array(1,:))
% grid();
% title('Force in surge (including. disturbance)');
% xlabel('t [s]');
% ylabel('Force [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Sway (no disturbance)
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.u_array(2,:))
% grid();
% title('Force in sway (no disturbance)');
% xlabel('t [s]');
% ylabel('Force [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Sway (including disturbance)
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.u_array(2,:))
% grid();
% title('Force in sway (including disturbance)');
% xlabel('t [s]');
% ylabel('Force [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Sway (no disturbance)
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.u_array(3,:))
% grid();
% title('Momentum in yaw (no disturbance)');
% xlabel('t [s]');
% ylabel('Momentum [Nm]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Sway (including disturbance)
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.u_array(3,:))
% grid();
% title('Momentum in yaw (including disturbance)');
% xlabel('t [s]');
% ylabel('Momentum [Nm]');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f2, 'Workspace\Plots\forces.png');
% 
% %%%%%%%%%%%%%%%%%%%%
% %%% Position NED %%%
% %%%%%%%%%%%%%%%%%%%%
% % f3 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f3 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(3, 2, "TileSpacing", "compact");
% 
% % Position north (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.x_array(1,1:length_time));
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.setpoint(1,1:length_time));
% grid();
% title('Position north (no disturbance)');
% xlabel('t [s]');
% ylabel('North, x [m]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Position north (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), dp_dist.x_array(1,1:length_time));
% plot(dp_dist.t_array(1:length_time), dp_dist.setpoint(1,1:length_time));
% grid();
% title('Position north (including disturbance)');
% xlabel('t [s]');
% ylabel('North, y [m]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Position east (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.x_array(2,1:length_time));
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.setpoint(2,1:length_time));
% grid();
% title('Position east (no disturbance)');
% xlabel('t [s]');
% ylabel('East, x [m]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Position east (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), dp_dist.x_array(2,1:length_time));
% plot(dp_dist.t_array(1:length_time), dp_dist.setpoint(2,1:length_time));
% grid();
% title('Position east (including disturbance)');
% xlabel('t [s]');
% ylabel('East, x [m]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Heading (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), rad2deg(dp_no_dist.x_array(3,1:length_time)));
% plot(dp_no_dist.t_array(1:length_time), rad2deg(dp_no_dist.setpoint(3,1:length_time)));
% grid();
% title('Heading (no disturbance)');
% xlabel('t [s]');
% ylabel('Heading, \psi [°]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Heading (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), rad2deg(dp_dist.x_array(3,1:length_time)));
% plot(dp_dist.t_array(1:length_time), rad2deg(dp_dist.setpoint(3,1:length_time)));
% grid();
% title('Heading (including disturbance)');
% xlabel('t [s]');
% ylabel('Heading, \psi [°]');
% legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f3, 'Workspace\Plots\setpoint_tracking.png');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% Velocity and angular momentum in BODY frame %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % f4 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f4 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(3, 2, "TileSpacing", "compact");
% 
% % Velocity surge (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.x_array(4,1:length_time));
% grid();
% title('Velocity surge (no disturbance)');
% xlabel('t [s]');
% ylabel('Velocity, u [m/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Velocity surge (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), dp_dist.x_array(4,1:length_time));
% grid();
% title('Velocity surge (including disturbance)');
% xlabel('t [s]');
% ylabel('Velocity, u [m/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Velocity sway (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.x_array(5,1:length_time));
% grid();
% title('Velocity sway (no disturbance)');
% xlabel('t [s]');
% ylabel('Velocity, v [m/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Velocity sway (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), dp_dist.x_array(5,1:length_time));
% grid();
% title('Velocity sway (including disturbance)');
% xlabel('t [s]');
% ylabel('Velocity, v [m/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Rotation yaw (no disturbance)
% nexttile;
% hold on
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.x_array(6,1:length_time));
% grid();
% title('Rotation in yaw (no disturbance)');
% xlabel('t [s]');
% ylabel('Angular rotation, r [rad/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Velocity sway (including disturbance)
% nexttile;
% hold on
% plot(dp_dist.t_array(1:length_time), dp_dist.x_array(6,1:length_time));
% grid();
% title('Rotation in yaw (including disturbance)');
% xlabel('t [s]');
% ylabel('Angular rotation, r [rad/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f4, 'Workspace\Plots\velocity.png');
% 
% %%%%%%%%%%%%%%%%%%%
% %%% Kalman gain %%%
% %%%%%%%%%%%%%%%%%%%
% % f5 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f5 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(1, 2, "TileSpacing", "compact");
% 
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), dp_no_dist.K_array(:,1:length_time));
% grid();
% title('Kalman gain values (no disturbance)');
% xlabel('t [s]');
% ylabel('Gain matrix entries');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.K_array(:,1:length_time));
% grid();
% title('Kalman gain values (including disturbance)');
% xlabel('t [s]');
% ylabel('Gain matrix entries');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f5, 'Workspace\Plots\kalman.png');
% 
% %%%%%%%%%%%%
% %%% Wind %%%
% %%%%%%%%%%%%
% % f6 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f6 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(5, 1, "TileSpacing", "compact");
% 
% % Absolute velocity
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.wind_abs(1:length_time));
% grid();
% title('Wind');
% xlabel('t [s]');
% ylabel('Velocity [m/s]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Angle of attack
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), rad2deg(dp_dist.wind_beta(1:length_time)));
% grid();
% title('Wind angle');
% xlabel('t [s]');
% ylabel('Degree [°]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Force in surge
% nexttile;
% plot(dp_dist.t_array(1:length_time), dp_dist.wind_force_array(1,1:length_time));
% grid();
% title('Force in surge');
% xlabel('t [s]');
% ylabel('Force [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Force in sway
% nexttile;
% plot(dp_dist.t_array(1:length_time), dp_dist.wind_force_array(2,1:length_time));
% grid();
% title('Force in sway');
% xlabel('t [s]');
% ylabel('F [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Momentum in yaw
% nexttile;
% plot(dp_dist.t_array(1:length_time), dp_dist.wind_force_array(3,1:length_time));
% grid();
% title('Momentum in yaw');
% xlabel('t [s]');
% ylabel('N [Nm]');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f6, 'Workspace\Plots\wind.png');
% 
% %%%%%%%%%%%%%%%
% %%% Current %%%
% %%%%%%%%%%%%%%%
% % f7 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f7 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(1, 2, "TileSpacing", "compact");
% 
% % Current in north direction
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.current_force(1,1:length_time));
% plot(dp_dist.t_array(1:length_time), dp_dist.x_est_array(7,1:length_time));
% grid();
% title('Current in north direction');
% xlabel('t [s]');
% ylabel('Force in North [N]');
% legend({'Current force', 'Est. current force'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Current in east direction
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.current_force(2,1:length_time));
% plot(dp_dist.t_array(1:length_time), dp_dist.x_est_array(8,1:length_time));
% grid();
% title('Current in east direction');
% xlabel('t [s]');
% ylabel('Force in east [N]');
% legend({'Current force', 'Est. current force'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f7, 'Workspace\Plots\current.png');
% 
% %%%%%%%%%%%%%
% %%% Waves %%%
% %%%%%%%%%%%%%
% % f8 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f8 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(3, 1, "TileSpacing", "compact");
% 
% % Wave force in north direction
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.wave_force(1,1:length_time))
% grid();
% title('Waves in north direction');
% xlabel('t [s]');
% ylabel('Force in north [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Wave force in east direction
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.wave_force(2,1:length_time))
% grid();
% title('Waves in east direction');
% xlabel('t [s]');
% ylabel('Force in east [N]');
% grid on, grid minor;
% box on;
% hold off;
% 
% % Wave momentum in yaw
% nexttile;
% hold on;
% plot(dp_dist.t_array(1:length_time), dp_dist.wave_force(3,1:length_time))
% grid();
% title('Wave momentum in yaw');
% xlabel('t [s]');
% ylabel('Force in yaw [Nm]');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f8, 'Workspace\Plots\waves.png');
% 
% %%%%%%%%%%%
% %%% IAE %%%
% %%%%%%%%%%%
% 
% % No disturbance
% error_no_dist = abs(dp_no_dist.setpoint(:,1:length_time) - dp_no_dist.x_array(1:3,1:length_time));
% 
% % Including disturbance
% error_dist = abs(dp_dist.setpoint(:,1:length_time) - dp_dist.x_array(1:3,1:length_time));
% 
% % Plot error error
% % f9 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
% f9 = figure('units', 'normalized', 'DefaultAxesFontSize', 18);
% t = tiledlayout(3, 1, "TileSpacing", "compact");
% 
% % Error in north direction
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_no_dist(1,:)));
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_dist(1,:)));
% grid();
% title('Cumulative error north');
% xlabel('t [s]');
% ylabel('Cumulative north error [m]');
% legend({'No disturbance', 'Including disturbance'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_no_dist(2,:)));
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_dist(2,:)));
% grid();
% title('Cumulative error east');
% xlabel('t [s]');
% ylabel('Cumulative east error [m]');
% legend({'No disturbance', 'Including disturbance'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% nexttile;
% hold on;
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_no_dist(3,:)));
% plot(dp_no_dist.t_array(1:length_time), cumsum(error_dist(3,:)));
% grid();
% title('Cumulative error in yaw');
% xlabel('t [s]');
% ylabel('Cumulative yaw error [rad]');
% legend({'No disturbance', 'Including disturbance'}, 'Location', 'Best');
% grid on, grid minor;
% box on;
% hold off;
% 
% saveas(f9, 'Workspace\Plots\cumulative_error.png');
% 
% disp("Integrated Absolute Error (IAE):");
% 
% % IAE
% iae_no_dist = sum(error_no_dist,2)
% iae_dist = sum(error_dist,2)
% 
% %%%%%%%%%%
% %%% TV %%%
% %%%%%%%%%%
% 
% disp("Total Value (TV):");
% 
% % No disturbance
% u_total_value_no_dist = sum(abs(dp_no_dist.u_array(:,1:length_time)),2)
% 
% % Including disturbance
% u_total_value_dist = sum(abs(dp_dist.u_array(:,1:length_time)),2)
