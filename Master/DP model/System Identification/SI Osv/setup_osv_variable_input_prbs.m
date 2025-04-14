%%% Setup for ship subjected to variable values in the control input
%%% using PRBS (PseudoRandom Binary Sequence)

dt = 0.1;           % Timestep used in integration

T = 7200;           % End time
N = ceil(T/dt);     % Number of sample steps

% Select integration method
% IntegrationMethod.Runge_Kutta_Fourth_Order
% IntegrationMethod.Forward_Euler
integration_method = IntegrationMethod.Runge_Kutta_Fourth_Order;

x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];      % Inital values of states

% Predefined control inputs
u_array = zeros(6,N);
u_force_array = zeros(3,N);

% Constant azimuth angles
azimuth_angle_1 = 0;
azimuth_angle_2 = 0;

% Important to differentiate between thruster RPS and azimuth angle.
% We want the azimuth angle pointed in a fixed direction,
% in this case pointing in negative surge direction, thus
% emulating the behaviour of rudderless propellers. The second
% tunnel thruster is set to be inactive (zero control signal).
rps_multiplier = [6000/60; 0; 6000/60; 6000/60];
Tmin = 50;
Tmax = 250;
for i=1:4
        [u_array(i,:),~] = prbs1(N, Tmin, Tmax);         % PseudoRandom Binary Sequence (function written by David De Ruscio (USN))
        u_array(i,:) = rps_multiplier(i)*u_array(i,:);
end
u_array(5,:) = deg2rad(azimuth_angle_1);                 % Azimuth thruster 1 points backwards
u_array(6,:) = deg2rad(azimuth_angle_2);                 % Azimuth thruster 2 points backwards

u_squared(1:4,:) = u_array(1:4,:).*abs(u_array(1:4,:));  % Force appears to be a linear transformation of the value u*abs(u)                                                                                                           

% Automatically generate state space model
generate_state_space_model = true;

% Calculate generalized force
% These formulas an values are fetched from the OSV script developed
% by Thor I. Fossen with some slight modifcations by me to obtain
% the relevant forces
L = 83;                                                         % Length (m)
K_max = [300e3 300e3 420e3 655e3]';                             % Max propeller thrust (N)
n_max = [140 140 150 200]';                                     % Max propeller speed (rpm)
K_thr = diag(K_max./n_max.^2);                                  % Thruster coefficient matrix
l_x = [37, 35, -L/2, -L/2];                                     % Thruster x-coordinates (m)
l_y = [0, 0, 7, -7];                                            % Thruster y-coordinates (m)

% If om the future alpha is to changed during the run, must be modified.
T_thr = thrConfig( {'T', 'T', deg2rad(azimuth_angle_1), deg2rad(azimuth_angle_1)}, l_x, l_y);

for i=1:N
    u_thr = abs(u_array(1:4,i)) .* u_array(1:4,i);
    tau_3dof = T_thr * K_thr * u_thr;                           % Quadratic propeller speed
    tau_thr = [ tau_3dof(1) tau_3dof(2) 0 0 0 tau_3dof(3) ]';

    % We are only interested in force/momentum with regards to u,v and r
    u_force_array(:,i) = [tau_thr(1); tau_thr(2); tau_thr(6)];
end

% Plots can be shown for debugging purposes       
show_debug_plots = true;

if (show_debug_plots)
    figure(101);
    titles = {'Tunnel thruster (1)', 'Tunnel thruster (2)', 'Azimuth thruster (3)', ...
        'Azimuth thruster (4)', 'Angle azimuth thruster (3)', 'Angle azimuth thruster (4)'};
    ylabels = {'RPM', 'RPM', 'RPM', 'RPM', 'Angle [°]', 'Angle [°]'};
    time = 0:dt:T+1;
    time = time(1:N);
    for i=1:6
        subplot(3,2,i);
        if ((i == 5) || (i == 6))
            plot(time, rad2deg(u_array(i,:)));
        else
            plot(time, u_array(i,:));
        end
        grid();
        title(titles(i));
        xlabel('t [s]');
        ylabel(ylabels(i));
    end
    
    figure(102);
    titles = {'Force surge', 'Force sway', 'Yaw momentum'};
    ylabels = {'Force [N]', 'Force [N]', 'Momentum [Mm]'};
    time = 0:dt:T+1;
    time = time(1:N);
    for i=1:3
        subplot(1,3,i);
        plot(time, u_force_array(i,:));
        grid();
        title(titles(i));
        xlabel('t [s]');
        ylabel(ylabels(i));
    end
end
