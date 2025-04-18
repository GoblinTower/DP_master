%%% Setup for ship subjected to variable values in the control input

dt = 0.01;          % Timestep used in integration

T = 500;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

x0 = [0; 0; 0; 0; 0; 0]; % Inital values of states
u0 = [5000; 5000; 5000];

% Predefined control inputs
u = zeros(3,N);

variance = [1000; 1000; 1000];

% Gaussian random walk
u(:,1) = u0;
for j=2:N
    u(:,j) = u(:,j-1) + normrnd(0, variance, 3, 1);
end

% Simple low pass filtering of signal
% Gives a more smooth transition in the control signal
% u_smoothed(k) = alpha*u_smoothed(k-1) + (1 - alpha)*u(k) 
% Source: https://en.wikipedia.org/wiki/Low-pass_filter#Difference_equation_through_discrete_time_sampling

alpha = 0.99;
for i=1:3
    u_smoothed = u(i,1);
    for j=2:N
        u_smoothed = u_smoothed*alpha + (1 - alpha)*u(i,j);
        u(i,j) = u_smoothed;
    end
end
