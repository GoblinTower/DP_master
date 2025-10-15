% File containing the the trajectory tracking setpoints used in all simulations.
% This is used in the MPC simulations.

%%%%%%%%%%%%%%%%%%
%%% Trajectory %%%
%%%%%%%%%%%%%%%%%%
% Creating waypoints
t = [0, 200, 400, 600, 800, 1000, 1200, 1400, 1600];
x = [0, 10, 20, 30, 40, -10, -2, -20, -20];
y = [0, 5, 15, 10, 10, 35, 60, 80, 80];

delta_t = t(2:end) - t(1:end-1);
delta_x = x(2:end) - x(1:end-1);
delta_y = y(2:end) - y(1:end-1);

xdot = [delta_x./delta_t, 0.0];
ydot = [delta_y./delta_t, 0.0];

% Calling trajectory function
[t_c, x_c, y_c, xdot_c, ydot_c] = cubic_interpolation(t, x, y, xdot, ydot, dt);

% Calculate yaw angle
diff_x = x_c(2:end) - x_c(1:end-1);
diff_y = y_c(2:end) - y_c(1:end-1);
yaw_setpoints = [atan2(diff_y, diff_x), atan2(diff_y(end), diff_x(end))];

setpoint = [x_c; y_c; yaw_setpoints];

% Add extended reference horizon (needed since NMPC predicts into the
% future).
setpoint = [setpoint, repmat(setpoint(:, end), 1, horizon_length - 1)];