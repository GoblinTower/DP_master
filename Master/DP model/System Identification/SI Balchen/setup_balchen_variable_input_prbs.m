%%% Setup for ship (Balchen model) subjected to variable values in the control input
%%% using PRBS (PseudoRandom Binary Sequence)

dt = 0.1;           % Timestep used in integration
T = 3000;           % End time
N = ceil(T/dt);     % Number of sample steps

n_si_dim = 3;       % Number of states in system identification

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

x0 = [0; 0; 0; 0; 0; 0; 0; 0];      % Inital values of states

% Predefined control inputs
u_array = zeros(3,N);

% Calculate thruster forces and momentum
force_multiplier = [1e5; 1e5; 5e6];
Tmin = 50;
Tmax = 100;
for i=1:3
    [u_array(i,:),~] = prbs1(N, Tmin, Tmax);             % PseudoRandom Binary Sequence (function written by David De Ruscio (USN))
    u_array(i,:) = force_multiplier(i)*u_array(i,:);
end

% Automatically generate state space model
generate_state_space_model = true;

% Plots can be shown for debugging purposes       
show_debug_plots = false;

if (show_debug_plots)
    
    figure(100);
    titles = {'Force surge', 'Force sway', 'Yaw momentum'};
    ylabels = {'Force [N]', 'Force [N]', 'Momentum [Mm]'};
    time = 0:dt:T+1;
    time = time(1:N);
    for i=1:3
        subplot(1,3,i);
        plot(time, u_array(i,:));
        grid();
        title(titles(i));
        xlabel('t [s]');
        ylabel(ylabels(i));
    end
end