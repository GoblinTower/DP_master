% Test script for verifying correctness of RK4 function
% It looks at the projectile problem. This is the simplest version,
% where the projectile is not subjected to other external forces
% with the exception of gravity. The formulas are given below:
%
% ddot_y = -g
% 
% This equation can be rewritten to the following form:
%
% dot_y = v
% dot_v = -g
%
% The parameters will be g = 9.81.
%
% The initial paremeters will be y = 10 and v = 20.
%
% The formula for height is x = v*t - 0.5*g*t^2 + x0. We want to find
% the point when it reaches ground, assumed to be x = 0. It can be shown
% that the projectile reaches the ground when t = (v0/g)*(1 + sqrt(1 + 2*x0*g/v0^2))
% This corresponds to t = 4.5277
%
clear, clc, close all;

addpath("..\");

% Parameters
g = 9.81;

% Initial values
x0 = 10;
v0 = 20;

% State vector
x = [x0; v0];

% Timespan
T = 5;
dt = 0.1;
N = floor(T/dt);
t = 0;

% Storing data
x_array = zeros(2,N);
t_array = zeros(1,N);

% Expected time reaching ground
t_ground = (v0/g)*(1 + sqrt(1 + 2*x0*g/v0^2));

for i=1:N
    x_array(:,i) = x;
    t_array(i) = t;

    % Integrate
    [~, x] = runge_kutta_4(@(t, x) projectile(t, x, g), t, x, dt);
    t = t + dt;
end

% Plot results
% Plot position
figure(1);
subplot(1,2,1);
hold on;
plot(t_array, x_array(1,:));
plot(t_ground, 0, 'ro');
title("Height [m]");
xlabel("Time [s]");
ylabel("Height [m]");
grid();
legend({"Height", "Expected time reaching ground"}, "Location", "Best");
hold off;

% Plot velocity
subplot(1,2,2);
plot(t_array, x_array(2,:));
title("Velocity [m/s]");
xlabel("Time [s]");
ylabel("Velocity [m/s]");
grid();

function x_dot = projectile(t, x, g)
    x_dot(1) = x(2);
    x_dot(2) = -g;
end
