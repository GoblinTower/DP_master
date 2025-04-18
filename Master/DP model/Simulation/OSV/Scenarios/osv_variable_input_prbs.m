%%% Setup for ship subjected to variable values in the control input
%%% using PRBS (PseudoRandom Binary Sequence)

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

Tmin = 100;
Tmax = 2000;
rpm_multiplier = [500; 500; 500; 500; 2*pi; 2*pi]; % Important to differentiate between thruster RPM and azimuth angle
for i=1:6
    [u(i,:),~] = prbs1(N, Tmin, Tmax); % PseudoRandom Binary Sequence (written by David De Ruscio (USN))
    u(i,:) = rpm_multiplier(i)*u(i,:);
end
