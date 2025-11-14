% Simple script for plotting and comparing LQ control simulations on supply
% model where underlying mathematical model is identified using different
% system identification techniques.
clear, clc, close all;

addpath("..\..\Tools\");

mpc_const_r_dist = load('Workspace\mpc_const_r_du_with_dist.mat');                             % b term starting as a zero vector
mpc_const_r_estimate = load('Workspace\mpc_const_r_du_with_dist_current_adjusted.mat');        % b term starting as a good estimate

storage_path = 'Workspace\b_term';                                                             % Path where plots are stored

fig_estimate = plot_b_comparison(mpc_const_r_estimate, 'b set true value' , 5, 5);
save_plot(fig_estimate, 'mpc_r_good_estimate_b', storage_path);

fig_estimate = plot_b_comparison(mpc_const_r_dist, 'b set to 0' , 5, 5);
save_plot(fig_estimate, 'mpc_r_zero_estimate_b', storage_path);

function fig = plot_b_comparison(mpc, comment, exponent_north, exponent_east)

    dp = get(groot, 'DefaultFigurePosition');   % Default position
    
    fig = figure('DefaultAxesFontSize', 18, 'Position', [dp(1), dp(2), dp(3), 0.7*dp(4)]);
    t = tiledlayout(2, 2, "TileSpacing", "compact");
    
    length_time = min(size(mpc.t_array,2), size(mpc.u_array,2));
    
    % North position
    nexttile;
    hold on;
    plot(mpc.t_array(1:length_time), mpc.x_array(1,1:length_time));
    plot(mpc.t_array(1:length_time), mpc.setpoint(1,1:length_time));
    grid();
    title(strcat('Position north (', comment, ')'));
    xlabel('t [s]');
    ylabel('North, x^n [m]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

    % Current in north direction
    nexttile;
    hold on;
    plot(mpc.t_array(1:length_time), mpc.current_force(1,1:length_time));
    plot(mpc.t_array(1:length_time), mpc.x_est_array(7,1:length_time));
    grid();
    title(strcat('Current north', ' (', comment, ')'));
    xlabel('t [s]');
    ylabel('Force in north [N]');
    legend({'Current force', 'Est. current force'}, 'Location', 'southeast');
    ax = gca;
    ax.YAxis.Exponent = exponent_north;
    grid on, grid minor;
    box on;
    ylim([0, 1.2e5]);
    hold off;
    
    % East position
    nexttile;
    hold on;
    plot(mpc.t_array(1:length_time), mpc.x_array(2,1:length_time));
    plot(mpc.t_array(1:length_time), mpc.setpoint(2,1:length_time));
    grid();
    title(strcat('Position east (', comment, ')'));
    xlabel('t [s]');
    ylabel('East, y^n [m]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

    % Current in east direction
    nexttile;
    hold on;
    plot(mpc.t_array(1:length_time), mpc.current_force(2,1:length_time));
    plot(mpc.t_array(1:length_time), mpc.x_est_array(8,1:length_time));
    grid();
    title(strcat('Current east', ' (', comment, ')'));
    xlabel('t [s]');
    ylabel('Force in east [N]');
    legend({'Current force', 'Est. current force'}, 'Location', 'southeast');
    ax = gca;
    ax.YAxis.Exponent = exponent_east;
    grid on, grid minor;
    box on;
    ylim([0, 2.3e5]);
    hold off;

end