%%% Setup for ship subjected to constant values in the control input

dt = 0.01;          % Timestep used in integration

T = 100;            % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % Inital values of states

% Predefined control inputs
u = zeros(6,N);
for i=1:N
    u(:,i) = [100; 200; -100; 50; 0.5*pi; 0];
end
