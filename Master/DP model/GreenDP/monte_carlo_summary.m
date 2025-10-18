% Simple script for investigating the results from the Monte Carlo
% simulation
clear, clc, close all;

addpath("..\..\Tools\");

number_of_simulations = 100;

% Select data
% folder = 'Monte_Carlo';
folder = 'Monte_Carlo_Reduction';

% Fetch data from first simulation, should be the same for all
sim1 = load(strcat('Workspace\', folder, '\nmpc_mc_green_dp_dist_data_1.mat'));

t_array = sim1.t_array;
number_if_simulation_steps = min(size(t_array,2), size(sim1.u_array,2));
reference = zeros(3, number_if_simulation_steps);

% Preallocate arrays:
u_green = zeros(3,number_if_simulation_steps, number_of_simulations);
u_no_green = zeros(3,number_if_simulation_steps, number_of_simulations);
x_green = zeros(6,number_if_simulation_steps+1, number_of_simulations);
x_no_green = zeros(6,number_if_simulation_steps+1, number_of_simulations);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Technical Value (TV) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read data from files
for iteration=1:number_of_simulations
    
    % Get data from greenDP simulations
    sim = load(strcat('Workspace\', folder, '\nmpc_mc_green_dp_dist_data_', num2str(iteration), '.mat'));
    u_green(:,:,iteration) = sim.u_array; 
    x_green(:,:,iteration) = sim.x_array; 

    % Get data from non-greenDP simulations
    sim = load(strcat('Workspace\', folder, '\nmpc_mc_no_green_dp_dist_data_', num2str(iteration), '.mat'));
    u_no_green(:,:,iteration) = sim.u_array;
    x_no_green(:,:,iteration) = sim.x_array;
end

% u_green = u_green(:,200:end,:);
% u_no_green = u_no_green(:,200:end,:);

% Get technical values (TV)
u_total_green = squeeze(sum(abs(u_green),2));
u_total_no_green = squeeze(sum(abs(u_no_green),2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot simulation results %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f1 = figure('DefaultAxesFontSize', 20, 'Position', [0, 0, 1200, 1600]);
f1 = figure('DefaultAxesFontSize', 20);
t = tiledlayout(3, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(u_total_green(1,:), 'r*');
plot(u_total_no_green(1,:), 'k*');
grid();
title('Integrated absolute value of force in surge');
xlabel('Simulation number');
ylabel('Time [s]');
legend({'GreenDP', 'Normal DP'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(u_total_green(2,:), 'r*');
plot(u_total_no_green(2,:), 'k*');
grid();
title('Integrated absolute value of force in sway');
xlabel('Simulation number');
ylabel('Time [s]');
legend({'GreenDP', 'Normal DP'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(u_total_green(3,:), 'r*');
plot(u_total_no_green(3,:), 'k*');
grid();
title('Integrated absolute value of moment in yaw');
xlabel('Simulation number');
ylabel('Time [s]');
legend({'GreenDP', 'Normal DP'}, 'Location', 'northeast');
grid on, grid minor;
box on;
hold off;

save_plot(f1, 'mc_results', strcat('MonteCarlo\', folder));

% Compute average technical value
avg_tv_green = mean(u_total_green,2);
disp("Average technical value GreenDP simulations: ");
disp(avg_tv_green);

avg_tv_no_green = mean(u_total_no_green,2);
disp("Average technical value normal DP simulations: ");
disp(avg_tv_no_green);

avg_percentage = (avg_tv_no_green - avg_tv_green)./avg_tv_no_green*100;
disp("GreenDP percentage fuel consumption compared to setpoint tracking: ");
disp(avg_percentage);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Average distance %%%
%%%%%%%%%%%%%%%%%%%%%%%%
distance_array_green = zeros(1, number_of_simulations);
distance_array_no_green = zeros(1, number_of_simulations);
for iter=1:number_of_simulations

    % Get average distance for green DP for each simulation
    distance_array_green(1,iter) = mean(sqrt(sum(x_green(1:2,:,iter).^2,1)));

    % Get average distance for normal DP for each simulation
    distance_array_no_green(1,iter) = mean(sqrt(sum(x_no_green(1:2,:,iter).^2,1)));

end

disp('Average distance for green DP');
disp(mean(distance_array_green));
disp('Average distance for normal DP');
disp(mean(distance_array_no_green));
