% File containing the the trajectory tracking setpoints used in all simulations.
% This is used in the MPC simulations.

%%%%%%%%%%%%%%%%%%
%%% Trajectory %%%
%%%%%%%%%%%%%%%%%%
% Creating waypoints
t = [0, 200, 400, 600, 800, 1000, 1200, 1400, 1600];
x = [0, 10, 20, 30, 40, -10, -2, -20, -20];
y = [0, 5, 15, 10, 10, 35, 60, 80, 80];

waypoints = [x; y];

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

% How to deal with the case when both diff_y and diff_x are zero,
% i.e. when the ship is not moving? Decide to keep the latest yaw angle
% that was last used
end_index = find(diff_x==0, 1, 'first') - 1;
number_of_indices = size(x_c, 2);
last_yaw = atan2(diff_y(end_index), diff_x(end_index));

yaw_setpoints = [atan2(diff_y(1:end_index), diff_x(1:end_index)), last_yaw*ones(1,number_of_indices - end_index)];

% Always choose shortest yaw change path
yaw_setpoints = find_closest_angle(yaw_setpoints);

setpoint = [x_c; y_c; yaw_setpoints];

% Add extended reference horizon (needed since NMPC predicts into the
% future).
setpoint = [setpoint, repmat(setpoint(:, end), 1, horizon_length - 1)];

run_debug = false;
if (run_debug)
    plot_trajectory(t_c, x_c, y_c, yaw_setpoints(:,1:number_of_indices), xdot_c, ydot_c);
end