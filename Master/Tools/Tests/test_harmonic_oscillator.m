% Test script for verifying correctness of RK4 function.
% Uses the well known simple harmonic oscillator as an example.
% The harmonic oscillator is a differential equation
% on the following form:
%
% m*ddot_x = -k*x 
%
% This equation is rewritten to the following form
%
% dot_x = v
% dot_v = -(k/m)*x 
%
% This results in sustained oscillations with fixed
% amplitude and frequency:
%
% w = sqrt(k/m)
% f = 1/(2*pi)*sqrt(k/m)
%
% We want to verify if the resulting solution from RK4
% behaves as expected. 
%
% Using m = 4, k = 1 gives: f = 1/(4*pi) frequency.
% The period of each wave should hence be T = 1/f = 4*pi
% T = 12.5664
%
% Assuming initial values are x0 = 5 and v0, we expect
% a sustained amplitude of 5. The example here can be
% thought of as a mass connected to string. There is no damping
% or friction.
%

% Parameters
k = 1;
m = 4;

% Initial values
x0 = 5;
v0 = 0;

% State vector
x = [x0; v0];

% Timespan
T = 50;
dt = 0.01;
N = floor(T/dt);
t = 0;

% Storing data
x_array = zeros(2,N);
t_array = zeros(1,N);

for i=1:N
    x_array(:,i) = x;
    t_array(i) = t;

    % Integrate
    [~, x] = runge_kutta_4(@(t, x) harmonic_oscillator(t, x, m, k), t, x, dt);
    t = t + dt;
end

% Plot results
% Plot position
figure(1);
subplot(1,2,1);
plot(t_array, x_array(1,:));
title("Position [m]");
xlabel("Time [s]");
ylabel("Amplitude [m]");
grid();

% Plot velocity
subplot(1,2,2);
plot(t_array, x_array(2,:));
title("Velocity [m/s]");
xlabel("Time [s]");
ylabel("Velocity [m/s]");
grid();

function x_dot = harmonic_oscillator(t, x, m, k)
    x_dot(1) = x(2);
    x_dot(2) = -(k/m)*x(1);
end
