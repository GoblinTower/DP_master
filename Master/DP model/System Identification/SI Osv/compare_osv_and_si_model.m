% Script for comparing surge velocity, sway velocity and heading change
% when running the same input values for both the OSV model and the
% identified model.
clear, clc, close all;

addpath("..\..\..\Tools\");

% Create model using DSR generated matrices
dsr = load('..\Log\ssm_dsr_osv.mat');
A_si = dsr.A;
B_si = dsr.B;
C_si = dsr.C;

n_dim = size(A_si,1);           % Number of states in SI model
m_dim = size(C_si,1);           % Number of outputs in SI model

% Simulation parameters
% Time step must be the same as when calculating discrete system matrices during
% system identification.
dt = 1.0;                                                           % Time step       
T = 100;                                                            % Time length of simulation
N = ceil(T/dt);                                                     % Number of simulation steps
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;    % Integration method applied to OSV

% Create input array using gaussian random walk
u_array = zeros(6,N);                                               % Array of inputs (OSV)
std = [1.0; 1.0; 1.0; 1.0; 0; 0];                                   % Variance of gaussian random walk
mean = [0; 0; 0; 0; 0; 0];                                          % Mean of gaussian random walk
u_array(:,1) = zeros(6,1);                                          % Initial value of gaussian random walk
for j=2:N
    u_array(:,j) = u_array(:,j-1) + normrnd(0, std, 6, 1);
end

% Initial values of states
x0 = zeros(12,1);               % Initial state vector OSV
x0_si = zeros(n_dim,1);         % Initial state vector SI

% Preallocate arrays
t = 0;
t_array = zeros(1,N+1);         % Time

x_array = zeros(12,N+1);        % State vector array OSV
x_array(:,1) = x0;              % Initialize first entry in state vector array OSV

x_si_array = zeros(n_dim,N+1);  % State vector array SI model
x_si_array(:,1) = x0_si;        % Initialize first entry in state vector array SI model

y_si_array = zeros(m_dim,N+1);  % Output vector array SI model
y_si_array(:,1) = C_si*x0_si;   % Initialize first entry in output vector array SI model

% Run simulations
x = x0;                         % State vector OSV
x_si = x0_si;                   % State vector SI model

for i=1:N

    % Update state OSV
    switch (integration_method)

        case (IntegrationMethod.Forward_Euler)
            % Forward Euler
            [xdot,U,M] = osv(x, u_array(:,i), 0, 0);
            x = x + xdot*dt;

        case (IntegrationMethod.Runge_Kutta_Fourth_Order)
            % Runge-Kutta 4th order
            [~, x] = runge_kutta_4(@(t, x) osv(x, u_array(:,i), 0, 0), t, x, dt);
    end

    % Convert to u to input to identified model
    u = u_array(:,i);
    u_mod = [u(1)*abs(u(1)); u(3)*abs(u(3)); u(4)*abs(u(4))];
    % u_mod = [u(1); u(3); u(4)];
    
    % Update SI
    x_si = A_si*x_si + B_si*u_mod;
    y_si = C_si*x_si;

    % Update time
    t = t + dt; 
    
    % Store data
    t_array(i+1) = t;
    x_array(:,i+1) = x;
    x_si_array(:,i+1) = x_si;
    y_si_array(:,i+1) = y_si;
    
end

% Plot data
plot_osv_si_comparison(t_array, u_array, x_array, y_si_array);
