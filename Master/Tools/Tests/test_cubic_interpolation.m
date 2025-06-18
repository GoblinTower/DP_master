% Test script for verifying cubic_interpolation() function
clear, clc, close all;

addpath("..\");

run_simulation = 2;

if (run_simulation == 1)

    % Creating waypoints
    t = [0, 5, 10, 20, 25, 36, 50];
    x = [0, 2, 4, 7, 4, 2, -2];
    y = [0, 4, 4, 6, 10, 15, 20];
    
    delta_t = t(2:end) - t(1:end-1);
    delta_x = x(2:end) - x(1:end-1);
    delta_y = y(2:end) - y(1:end-1);
    
    xdot = [delta_x./delta_t, 0.5];
    ydot = [delta_y./delta_t, 0.5];
    % xdot = [0.5, 0.5, 0.5, 0.2, -0.2, -0.1, -0.4];
    % ydot = [1.0, 1.0, 0, 0.4, 0.8, 1.0, 1.0, 1.1];

    % Sampling time
    dt = 0.2;

elseif (run_simulation == 2)
    
    % Creating waypoints
    t = [0, 200, 400, 600, 800, 1000, 1200, 1400, 1600];
    x = [0, 10, 20, 30, 40, -10, -2, -20, -20];
    y = [0, 5, 15, 10, 10, 35, 60, 80, 80];
    
    delta_t = t(2:end) - t(1:end-1);
    delta_x = x(2:end) - x(1:end-1);
    delta_y = y(2:end) - y(1:end-1);
    
    xdot = [delta_x./delta_t, 0.0];
    ydot = [delta_y./delta_t, 0.0];

    % Sampling time
    dt = 1.0;
end

% Calling trajectory function
[t_c, x_c, y_c, xdot_c, ydot_c] = cubic_interpolation(t, x, y, xdot, ydot, dt);

% Plot trajectory
figure(1);
subplot(4,2,1:4);
hold on;
plot(y_c, x_c);
plot(y, x, 'kx');
plot(y_c(1), x_c(1), 'ro');
plot(y_c(end), x_c(end), 'bo');
title("Position (north/east)");
xlabel("Position east [m]");
ylabel("Position north [m]");
grid();
xlim([min(y_c)-2,max(y_c)+2]);
ylim([min(x_c)-2,max(x_c)+2]);
legend({"Position", "Waypoints" , "Start position", "End position"}, "Location", "Best");
hold off;

% North position trajectory
subplot(4,2,5);
plot(t_c, x_c);
title("Position north [m]");
ylabel("Position north [m]");
xlabel("Time [t]");

% East position trajectory
subplot(4,2,6);
plot(t_c, y_c);
title("Position east [m]");
ylabel("Position east [m]");
xlabel("Time [t]");

% North velocity
subplot(4,2,7);
plot(t_c, xdot_c);
title("Velocity north [m/s]");
ylabel("Position north [m]");
xlabel("Time [t]");

% East velocity
subplot(4,2,8);
plot(t_c, ydot_c);
title("Velocity east [m/s]");
ylabel("Position east [m]");
xlabel("Time [t]");
