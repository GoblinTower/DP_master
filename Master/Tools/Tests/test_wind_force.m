% Simple test for verifying the grouping function works appropriately
clear, clc, close all;

addpath("..\");

%%%%%%%%%%%%%%%%%%
%%% Parameters %%%
%%%%%%%%%%%%%%%%%%
% Wind parameters
rho = 1.247;        % [kg/m^3] - This is air density at 10 degrees
Af = 180.0;         % Frontal projected area 
Al = 311.0;         % Lateral projected area
L = 76.2;           % Length overall (total length from bow to stern)
Cx = 0.7;           % Wind coefficient with respect to surge
Cy = 0.825;         % Wind coefficient with respect to sway
Cn = 0.125;         % Wind coefficient with respect to yaw

%%%%%%%%%%%%
%%% Test %%%
%%%%%%%%%%%%
vessel_heading = deg2rad(0);        % Vessel heading
u = 0;                              % Surge velocity
v = 0;                              % Sway velocity

wind_velocity = 10;
number_of_iterations = 360;
angle_array = zeros(1, number_of_iterations);
wind_force_array = zeros(3, number_of_iterations);
for i=1:number_of_iterations
    angle_attack = deg2rad((i/number_of_iterations)*360);
    angle_array(1,i) = angle_attack;
    wind_force_array(:,i) = wind_force_calc(wind_velocity, angle_attack, vessel_heading, ...
        u, v, rho, Af, Al, L, Cx, Cy, Cn);
end

fig = figure('DefaultAxesFontSize', 20);
tl = tiledlayout(3, 1, "TileSpacing", "compact");

nexttile;
hold on;
plot(rad2deg(angle_array(1,:)), wind_force_array(1,:));
grid();
title('Surge force generation from different wind angles');
xlabel('\beta [°]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(rad2deg(angle_array(1,:)), wind_force_array(2,:));
grid();
title('Sway force generation from different wind angles');
xlabel('\beta [°]');
ylabel('Force [N]');
grid on, grid minor;
box on;
hold off;

nexttile;
hold on;
plot(rad2deg(angle_array(1,:)), wind_force_array(3,:));
grid();
title('Momentum generation from different wind angles');
xlabel('\beta [°]');
ylabel('Momentum [Nm]');
grid on, grid minor;
box on;
hold off;
