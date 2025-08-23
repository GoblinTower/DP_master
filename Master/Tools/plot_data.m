function [iae, tv] = plot_data(lmf, nos, storage_path, show_setpoints)
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

% Create folder if it does not exists
if (not(isfolder(storage_path)))
    mkdir(storage_path);
end

% Check that length of 'lmf' and 'name_of_simulations' are equal.
assert(length(lmf) == length(nos), ...
    'Error: name of simulations and number of .mat files must be equal');
number_of_simulations = length(lmf);

%%%%%%%%%%%%%%%%%%
%%% Path plots %%%
%%%%%%%%%%%%%%%%%%
f1 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, number_of_simulations, "TileSpacing", "compact");

for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).x_array(2,:), lmf(i).x_array(1,:))
    plot(lmf(i).x_array(2,1), lmf(i).x_array(1,1), 'ko', 'MarkerSize', 10, 'LineWidth', 3);      % Start position
    plot(lmf(i).x_array(2,end), lmf(i).x_array(1,end), 'ro', 'MarkerSize', 10, 'LineWidth', 3);  % End position
    grid();
    title(strcat('Path (', nos(i), ')'));
    xlabel('East [m]');
    ylabel('North [m]');
    xlim([-7,7]);
    ylim([-7,12]);
    legend({'Path', 'Start position', "End position"}, 'Location', 'Best');
    grid on, grid minor;
    box on;
    hold off;

end

saveas(f1, strcat(storage_path,'\path.png'));

%%%%%%%%%%%%%%
%%% Forces %%%
%%%%%%%%%%%%%%
f2 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "compact");

length_time = min(size(lmf(1).t_array,2), size(lmf(1).u_array,2));

% Surge
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_array(1,:))
    grid();
    title(strcat('Force in surge (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Force [N]');
    grid on, grid minor;
    box on;
    hold off;

end

% Sway
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_array(2,:))
    grid();
    title(strcat('Force in sway (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Force [N]');
    grid on, grid minor;
    box on;
    hold off;

end

% Momentum
for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).u_array(3,:))
    grid();
    title(strcat('Momentum in yaw (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Momentum [Nm]');
    grid on, grid minor;
    box on;
    hold off;

end

saveas(f2, strcat(storage_path,'\forces.png'));

%%%%%%%%%%%%%%%%%%%%
%%% Position NED %%%
%%%%%%%%%%%%%%%%%%%%
f3 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "compact");

% Position north
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(1,1:length_time));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), lmf(i).setpoint(1,1:length_time));
        legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Position north (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('North, x [m]');
    grid on, grid minor;
    box on;
    hold off;

end

% Position east
for i=1:number_of_simulations
        
    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(2,1:length_time));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), lmf(i).setpoint(2,1:length_time));
        legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Position east (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('East, x [m]');
    grid on, grid minor;
    box on;
    hold off;

end

% Heading
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), rad2deg(lmf(i).x_array(3,1:length_time)));
    if (show_setpoints)
        plot(lmf(i).t_array(1:length_time), rad2deg(lmf(i).setpoint(3,1:length_time)));
        legend({'Vessel', 'Setpoint'}, 'Location', 'Best');
    end
    grid();
    title(strcat('Heading (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Heading, \psi [°]');
    grid on, grid minor;
    box on;
    hold off;

end

saveas(f3, strcat(storage_path,'\setpoint_tracking.png'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity and angular momentum in BODY frame %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f4 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, number_of_simulations, "TileSpacing", "compact");

% Velocity surge
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(4,1:length_time));
    grid();
    title(strcat('Velocity surge (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Velocity, u [m/s]');
    grid on, grid minor;
    box on;
    hold off;

end

% Velocity sway
for i=1:number_of_simulations

    nexttile;
    hold on
    plot(lmf(i).t_array(1:length_time), lmf(i).x_array(5,1:length_time));
    grid();
    title(strcat('Velocity sway (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Velocity, v [m/s]');
    grid on, grid minor;
    box on;
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
    ylabel('Angular rotation, r [rad/s]');
    grid on, grid minor;
    box on;
    hold off;

end

saveas(f4, strcat(storage_path,'\velocity.png'));

%%%%%%%%%%%%%%%%%%%
%%% Kalman gain %%%
%%%%%%%%%%%%%%%%%%%
f5 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, number_of_simulations, "TileSpacing", "compact");

for i=1:number_of_simulations

    nexttile;
    hold on;
    plot(lmf(i).t_array(1:length_time), lmf(i).K_array(:,1:length_time));
    grid();
    title(strcat('Kalman gain values (', nos(i), ')'));
    xlabel('t [s]');
    ylabel('Gain matrix entries');
    grid on, grid minor;
    box on;
    hold off;

end

saveas(f5, strcat(storage_path,'\kalman.png'));

%%%%%%%%%%%%
%%% Wind %%%
%%%%%%%%%%%%
f6 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(5, 1, "TileSpacing", "compact");

% Absolute velocity
nexttile;
hold on;
plot(lmf(i).t_array(1:length_time), lmf(i).wind_abs(1:length_time));
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
plot(lmf(i).t_array(1:length_time), rad2deg(lmf(i).wind_beta(1:length_time)));
grid();
title('Wind angle');
xlabel('t [s]');
ylabel('Degree [°]');
grid on, grid minor;
box on;
hold off;

% Force in surge
nexttile;
plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(1,1:length_time));
grid();
title('Force in surge');
xlabel('t [s]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

% Force in sway
nexttile;
plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(2,1:length_time));
grid();
title('Force in sway');
xlabel('t [s]');
ylabel('F [N]');
grid on, grid minor;
box on;
hold off;

% Momentum in yaw
nexttile;
plot(lmf(i).t_array(1:length_time), lmf(i).wind_force_array(3,1:length_time));
grid();
title('Momentum in yaw');
xlabel('t [s]');
ylabel('N [Nm]');
grid on, grid minor;
box on;
hold off;

saveas(f6, strcat(storage_path,'\wind.png'));

%%%%%%%%%%%%%%%
%%% Current %%%
%%%%%%%%%%%%%%%
f7 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(1, 2, "TileSpacing", "compact");

% Current in north direction
nexttile;
hold on;
plot(lmf(i).t_array(1:length_time), lmf(i).current_force(1,1:length_time));
plot(lmf(i).t_array(1:length_time), lmf(i).x_est_array(7,1:length_time));
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
plot(lmf(i).t_array(1:length_time), lmf(i).current_force(2,1:length_time));
plot(lmf(i).t_array(1:length_time), lmf(i).x_est_array(8,1:length_time));
grid();
title('Current in east direction');
xlabel('t [s]');
ylabel('Force in east [N]');
legend({'Current force', 'Est. current force'}, 'Location', 'Best');
grid on, grid minor;
box on;
hold off;

saveas(f7, strcat(storage_path,'\current.png'));

%%%%%%%%%%%%%
%%% Waves %%%
%%%%%%%%%%%%%
f8 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 1, "TileSpacing", "compact");

% Wave force in north direction
nexttile;
hold on;
plot(lmf(i).t_array(1:length_time), lmf(i).wave_force(1,1:length_time))
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
plot(lmf(i).t_array(1:length_time), lmf(i).wave_force(2,1:length_time))
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
plot(lmf(i).t_array(1:length_time), lmf(i).wave_force(3,1:length_time))
grid();
title('Wave momentum in yaw');
xlabel('t [s]');
ylabel('Force in yaw [Nm]');
grid on, grid minor;
box on;
hold off;

saveas(f8, strcat(storage_path,'\waves.png'));

%%%%%%%%%%%
%%% IAE %%%
%%%%%%%%%%%

error = zeros(3, length_time, number_of_simulations);
iae = zeros(3, number_of_simulations);
tv = zeros(3, number_of_simulations);

for i=1:number_of_simulations
    error(:,:,i) = abs(lmf(i).setpoint(:,1:length_time) - lmf(i).x_array(1:3,1:length_time));
end

% Plot cumulative error
f9 = figure('units', 'normalized', 'outerposition', [0 0 1 1], 'DefaultAxesFontSize', 18);
t = tiledlayout(3, 1, "TileSpacing", "compact");

% Error in north direction
nexttile;
hold on;
for i=1:number_of_simulations
    plot(lmf(i).t_array(1:length_time), cumsum(error(1,:,i)));
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

saveas(f9, strcat(storage_path,'\cumulative_error.png'));

disp("Integrated Absolute Error (IAE):");

% IAE
for i=1:number_of_simulations
    
    iae(:,i) = sum(error(:,:,i),2);
    disp(strcat('IAE for ' + string(nos(i)) + ':'));
    disp(iae(:,i));

end

%%%%%%%%%%
%%% TV %%%
%%%%%%%%%%

disp("Total Value (TV):");

for i=1:number_of_simulations
    
    tv(:,i) = sum(abs(lmf(i).u_array(:,1:length_time)),2);
    disp('TV for ' + string(nos(i)) + ':');
    disp(tv(:,i));

end
