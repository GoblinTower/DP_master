% Plots result from single DP

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Show effect of new external disturbance %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
f1 = figure('DefaultAxesFontSize', 20);
t = tiledlayout(3, 2, "TileSpacing", "compact");

use_reduction_factor = false;

if (use_reduction_factor)
    plots_to_display = [1, 3, 30];
    folder = 'Monte_Carlo_reduction';
else
    plots_to_display = [1, 3, 30];
    folder = 'Monte_Carlo';
end

n_plots = size(plots_to_display, 2);

for i=1:n_plots

    %%%%%%%%%%%%%%%%%%%%
    %%% Plot GreenDP %%%
    %%%%%%%%%%%%%%%%%%%%
    % Fetch data
    sim = load(strcat('Workspace\', folder, '\nmpc_mc_green_dp_dist_data_', num2str(plots_to_display(i)) , '.mat'));

    nexttile;
    hold on;
    
    % Plot circle
    radius = 5;
    center = [0, 0];
    t_circle = 0:0.01:2*pi;
    x_circle = radius*cos(t_circle) + center(1);
    y_circle = radius*sin(t_circle) + center(2);
    plot(x_circle, y_circle, 'k--');

    plot(sim.x_array(2,:), sim.x_array(1,:), 'b');
    grid();
    title('Vessel path (green DP)');
    xlabel('East [m]');
    ylabel('North [m]');
    % legend({'Border', 'Vessel movement'}, 'Location', 'Best');
    xlim([-8,8]);
    ylim([-8,8]);
    axis equal;
    grid on, grid minor;
    box on;
    hold off;

    %%%%%%%%%%%%%%%%%%%%%%
    %%% Plot Normal DP %%%
    %%%%%%%%%%%%%%%%%%%%%%
    % Fetch data
    sim = load(strcat('Workspace\', folder, '\nmpc_mc_no_green_dp_dist_data_', num2str(plots_to_display(i)) , '.mat'));

    nexttile;
    hold on;
    plot(sim.x_array(2,:), sim.x_array(1,:));
    grid();
    title('Vessel path (normal DP)');
    xlabel('East [m]');
    ylabel('North [m]');
    xlim([-0.5,0.5]);
    ylim([-0.5,0.5]);
    axis equal;
    grid on, grid minor;
    box on;
    hold off;

end

save_plot(f1, 'path_plots', strcat('MonteCarlo\', folder));