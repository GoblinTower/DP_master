function [iae, tv] = plot_data_extended_osv(lmf, nos, show_setpoints, font_size, folder, details, show_current_estimate, plot_number_environmental)
% Plotting function that allows for some degree of modification through function arguments.
% The .mat files must be generated in same way as in the simulation files in this project.
%
% INPUT:
% lmf                : Shortcut for List of .Mat Files, imf = [data1.mat, data2.mat, ...]
% nos                : Shortcut for Name Of Simulations, nos = ['nmpc','lq', ...]
% storage_path       : Path where plots are stored
% show_setpoints     : Show setpoints in relevant plots
%
% OUTPUT:
% iae                : Integrated Absolute Error (IAE)
% tv                 : Total Value (TV)
%

% Check that length of 'lmf' and 'name_of_simulations' are equal.
assert(length(lmf) == length(nos), ...
    'Error: name of simulations and number of .mat files must be equal');
number_of_simulations = length(lmf);

dp = get(groot, 'DefaultFigurePosition');   % Default position

%%%%%%%%%%%%%%%%%%
%%% Path plots %%%
%%%%%%%%%%%%%%%%%%
f1 = figure('DefaultAxesFontSize', 20, 'Position', [dp(1), dp(2), dp(3), 0.7*dp(4)]);
t = tiledlayout(1, number_of_simulations, "TileSpacing", "compact");

for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).x_array(8,:), lmf(i).x_array(7,:))
    plot(lmf(i).x_array(8,1), lmf(i).x_array(7,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
    plot(lmf(i).x_array(8,end), lmf(i).x_array(7,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
    grid();
    title(strcat('Path (', nos(i), ')'));
    xlabel('East [m]');
    ylabel('North [m]');
    xlim(details.path_xlim);
    ylim(details.path_ylim);
    legend({'Path', 'Start position', "End position"}, 'Location', 'northwest');
    grid on, grid minor;
    box on;
    hold off;

end

save_plot(f1, 'path', folder);

%%%%%%%%%%%%%%
%%% Forces %%%
%%%%%%%%%%%%%%
f2 = figure('DefaultAxesFontSize', 13);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "tight");

length_time = min(size(lmf(1).t_array,2), size(lmf(1).u_array,2));

% Surge
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_generalized(1,:))
    grid();
    title(strcat('Force in surge (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Force [N]');
    ax = gca;
    ax.YAxis.Exponent = details.force_surge_exponent;
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Sway
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_generalized(2,1:length_time))
    grid();
    title(strcat('Force in sway (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Force [N]');
    ax = gca;
    ax.YAxis.Exponent = details.force_sway_exponent;
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Moment
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_generalized(3,1:length_time))
    grid();
    title(strcat('Moment in yaw (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Moment [Nm]');
    ax = gca;
    ax.YAxis.Exponent = details.momentum_yaw_exponent;
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

save_plot(f2, 'forces', folder);

%%%%%%%%%%%%%%%%%%%%
%%% Position NED %%%
%%%%%%%%%%%%%%%%%%%%
f3 = figure('DefaultAxesFontSize', font_size);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "tight");

% Position north
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(7,1:length_time));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), lmf(i).setpoint(1,1:length_time));
        % legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Position north (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('North, x^n [m]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Position east
for i=1:number_of_simulations
        
    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(8,1:length_time));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), lmf(i).setpoint(2,1:length_time));
        % legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Position east (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('East, y^n [m]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Heading
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), rad2deg(lmf(i).x_array(12,1:length_time)));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), rad2deg(lmf(i).setpoint(3,1:length_time)));
        % legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Heading (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Heading, \psi [°]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off; 

end

save_plot(f3, 'setpoint_tracking', folder);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity and angular momentum in BODY frame %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f4 = figure('DefaultAxesFontSize', font_size);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "tight");

% Velocity surge
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(1,1:length_time));
    grid();
    title(strcat('Velocity surge (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Velocity, u [m/s]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Velocity sway
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(6,1:length_time));
    grid();
    title(strcat('Velocity sway (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Velocity, v [m/s]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

% Rotation yaw
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(6,1:length_time));
    grid();
    title(strcat('Rotation in yaw (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Angular velocity, r [rad/s]');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

save_plot(f4, 'velocity', folder);

%%%%%%%%%%%%%%%%%%%
%%% Kalman gain %%%
%%%%%%%%%%%%%%%%%%%
f5 = figure('DefaultAxesFontSize', 14, 'Position', [dp(1), dp(2), dp(3), 0.7*dp(4)]);
t = tiledlayout(1, number_of_simulations, "TileSpacing", "tight");

for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).K_array(:,1:length_time));
    grid();
    title(strcat('Kalman gain (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Gain matrix entries');
    ax = gca;
    ax.YAxis.Exponent = details.kalman_exponent;
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

end

save_plot(f5, 'kalman', folder);

%%%%%%%%%%%%
%%% Wind %%%
%%%%%%%%%%%%
% f6 =  figure('DefaultAxesFontSize', font_size);
% t = tiledlayout(2, 1, "TileSpacing", "compact");

% Absolute velocity
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).wind_abs(1:length_time));
% grid();
% title('Wind velocity');
% xlabel('t [s]');
% ylabel('Velocity [m/s]');
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;

% Angle of attack
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), rad2deg(lmf(plot_number_environmental).wind_beta(1:length_time)));
% grid();
% title('Wind angle');
% xlabel('t [s]');
% ylabel('Degree [°]');
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;

% Force in surge
% nexttile;
% plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(1,1:length_time));
% grid();
% title('Force in surge');
% xlabel('t [s]');
% ylabel('Force [N]');
% ax = gca;
% ax.YAxis.Exponent = details.wind_surge_exponent;
% grid on, grid minor;
% box on;
% hold off;
% 
% Force in sway
% nexttile;
% plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(2,1:length_time));
% grid();
% title('Force in sway');
% xlabel('t [s]');
% ylabel('F [N]');
% ax = gca;
% ax.YAxis.Exponent = details.wind_sway_exponent;
% grid on, grid minor;
% box on;
% hold off;
% 
% Moment in yaw
% nexttile;
% plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(3,1:length_time));
% grid();
% title('Moment in yaw');
% xlabel('t [s]');
% ylabel('N [Nm]');
% ax = gca;
% ax.YAxis.Exponent = details.wind_momentum_exponent;
% grid on, grid minor;
% box on;
% hold off;

% save_plot(f6, 'wind', folder);

%%%%%%%%%%%%%%%
%%% Current %%%
%%%%%%%%%%%%%%%
% f7 = figure('DefaultAxesFontSize', font_size);
% t = tiledlayout(1, 2, "TileSpacing", "compact");
% 
% Current in north direction
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).current_force(1,1:length_time));
% if (show_current_estimate)
%     plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).x_est_array(7,1:length_time));
% end
% grid();
% title('Current in north direction');
% xlabel('t [s]');
% ylabel('Force in north [N]');
% if (show_current_estimate)
%     legend({'Current force', 'Est. current force'}, 'Location', 'Best');
% end
% ax = gca;
% ax.YAxis.Exponent = details.current_north_exponent;
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;
% 
% Current in east direction
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).current_force(2,1:length_time));
% if (show_current_estimate)
%     plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).x_est_array(8,1:length_time));
% end
% grid();
% title('Current in east direction');
% xlabel('t [s]');
% ylabel('Force in east [N]');
% if (show_current_estimate)
%     legend({'Current force', 'Est. current force'}, 'Location', 'Best');
% end
% ax = gca;
% ax.YAxis.Exponent = details.current_east_exponent;
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;
% 
% save_plot(f7, 'current', folder);

%%%%%%%%%%%%%
%%% Waves %%%
%%%%%%%%%%%%%
% f8 = figure('DefaultAxesFontSize', font_size);
% t = tiledlayout(3, 1, "TileSpacing", "compact");

% Wave force in north direction
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).wave_force(1,1:length_time))
% grid();
% title('Waves in north direction');
% xlabel('t [s]');
% ylabel('Force in north [N]');
% ax = gca;
% ax.YAxis.Exponent = details.wave_north_exponent;
% grid on, grid minor;
% box on;ylim('padded');
% hold off;

% Wave force in east direction
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).wave_force(2,1:length_time))
% grid();
% title('Waves in east direction');
% xlabel('t [s]');
% ylabel('Force in east [N]');
% ax = gca;
% ax.YAxis.Exponent = details.wave_east_exponent;
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;

% Wave moment in yaw
% nexttile;
% hold on;
% plot(lmf(plot_number_environmental).t_array(1:length_time), lmf(plot_number_environmental).wave_force(3,1:length_time))
% grid();
% title('Waves in yaw');
% xlabel('t [s]');
% ylabel('Moment in yaw [Nm]');
% ax = gca;
% ax.YAxis.Exponent = details.wave_momentum_exponent;
% grid on, grid minor;
% box on;
% ylim('padded');
% hold off;
% 
% save_plot(f8, 'waves', folder);

%%%%%%%%%%%
%%% IAE %%%
%%%%%%%%%%%

error = zeros(3, length_time, number_of_simulations);
iae = zeros(3, number_of_simulations);
tv = zeros(3, number_of_simulations);

for i=1:number_of_simulations
    error(:,:,i) = abs(lmf(i).setpoint(:,1:length_time) - lmf(i).x_array([7,8,12],1:length_time));
end

% Plot cumulative error
f9 = figure('DefaultAxesFontSize', font_size);
t = tiledlayout(3, 1, "TileSpacing", "compact");

% Error in north direction
nexttile;
hold on;
for i=1:number_of_simulations
    plot(lmf(i).t_array(1:length_time), cumsum(error(1,:,i)));
end
grid();
title('Cumulative error north');
xlabel('t [s]');
ylabel('Cumulative north error [m]');
legend(nos, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
for i=1:number_of_simulations
    plot(lmf(i).t_array(1:length_time), cumsum(error(2,:,i)));
end
grid();
title('Cumulative error east');
xlabel('t [s]');
ylabel('Cumulative east error [m]');
legend(nos, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
for i=1:number_of_simulations
    plot(lmf(i).t_array(1:length_time), cumsum(error(3,:,i)));
end
grid();
title('Cumulative error in yaw');
xlabel('t [s]');
ylabel('Cumulative yaw error [rad]');
legend(nos, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

save_plot(f9, 'cumulative_error', folder);

disp("Integrated Absolute Error (IAE):");

% IAE
for i=1:number_of_simulations
    
    iae(:,i) = sum(error(:,:,i),2)*lmf(i).dt;
    disp(strcat('IAE for ' + string(nos(i)) + ':'));
    disp(iae(:,i));

end

%%%%%%%%%%
%%% TV %%%
%%%%%%%%%%

disp("Total Value (TV):");

for i=1:number_of_simulations
    
    tv(:,i) = sum(abs(lmf(i).u_array(:,1:length_time)),2)*lmf(i).dt;
    disp('TV for ' + string(nos(i)) + ':');
    disp(tv(:,i));

end
