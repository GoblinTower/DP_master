% Simple script for visually verifying smoothness of
% smooth_cost_function(r)
clear, clc, close all;

addpath("..\");

% Defining weights
w1 = 5;
w2 = 10;

% Border 
r1 = 5;

% Incremental distances
dr = 0.01;

% Time array
r_array = 0:dr:15;

% Array size
n = size(r_array, 2);

% Allocate cost array
cost_array = zeros(1,n);

% Evaluate function
for i=1:n
    cost_array(i) = smooth_cost_function(r_array(i), r1, w1, w2);
end

% plot function
plot(r_array, cost_array);
xlabel("Distance [m]");
ylabel("Cost function");
grid();
