% Plotting function that visualizes current realizations

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Show effect of new external disturbance %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
f1 = figure('DefaultAxesFontSize', 20);
t = tiledlayout(3, 1, "TileSpacing", "compact");

plots_to_display = [1, 3, 15];
n_plots = size(plots_to_display, 2);

for i=1:n_plots

    % Fetch data
    sim = load(strcat('Workspace\Monte_Carlo\nmpc_mc_green_dp_dist_data_', num2str(plots_to_display(i)) , '.mat'));

    nexttile;
    hold on;
    plot(sim.current_force(1,:));
    plot(sim.current_force(2,:));
    grid();
    title('Example of current disturbances');
    xlabel('Time [s]');
    ylabel('Force [N]');
    legend({'North', 'East'}, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    hold off;

end

save_plot(f1, 'disturbance_examples', 'MonteCarlo\')