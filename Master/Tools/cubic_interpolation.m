function [t_c, x_c, y_c, xdot_c, ydot_c] = cubic_interpolation(t, x, y, xdot, ydot, dt)
% Implementation of cubic interpolation
% INPUT
% t                  : Waypoint time array (t_c(1) must be zero)
% x                  : Waypoint north position array
% y                  : Waypoint east position array
% xdot               : Waypoint north velocity array
% ydot               : Waypoint east velocity array
% dt                 : Sampling time
%
% OUTPUT
% t_c                : Trajectory time array
% x_c                : Trajectory position north array
% y_c                : Trajectory position east array
% xdot_c             : Trajectory velocity north array
% ydot_c             : Trajectory velocity east array

% Algorithm uses row vectors
if iscolumn(t)
    t = t';
end
if iscolumn(x)
    x = x';
end
if iscolumn(y)
    y = y';
end
if iscolumn(xdot)
    xdot = xdot';
end
if iscolumn(ydot)
    ydot = ydot';
end

number_of_waypoints = size(t, 2);           % Number of waypoints

t_c = t(1):dt:t(end);                       % Trajectory array
t_c_indices = (floor(t/dt)+1);              % Indices of waypoints in trajectory array
t_c_waypoint = t_c(t_c_indices);            % Waypoint time values (rounded down)

n_trajectory_points = size(t_c, 2);         % Number of trajectory points

% Allocating arrays
x_c = zeros(1, n_trajectory_points);        % North position trajectory array
y_c = zeros(1, n_trajectory_points);        % East position trajectory array
xdot_c = zeros(1, n_trajectory_points);     % North velocity trajectory array
ydot_c = zeros(1, n_trajectory_points);     % East velocity trajectory array

% X direction
for k=1:number_of_waypoints-1
    % Calculate coefficients
    T = polynomial_matrix(t_c_waypoint(k), t_c_waypoint(k+1));
    b = [x(k); x(k+1); xdot(k); xdot(k+1)];
    a = T\b;

    % Fill arrays
    time = t_c(t_c_indices(k):t_c_indices(k+1));
    x_c(t_c_indices(k):t_c_indices(k+1)) = a(1) + a(2)*time + a(3)*time.^2 + a(4)*time.^3;
    xdot_c(t_c_indices(k):t_c_indices(k+1)) = 0 + a(2) + 2*a(3)*time + 3*a(4)*time.^2;
end

% Y direction
for k=1:number_of_waypoints-1
    % Calculate coefficients
    T = polynomial_matrix(t_c_waypoint(k), t_c_waypoint(k+1));
    b = [y(k); y(k+1); ydot(k); ydot(k+1)];
    a = T\b;

    % Fill arrays
    time = t_c(t_c_indices(k):t_c_indices(k+1));
    y_c(t_c_indices(k):t_c_indices(k+1)) = a(1) + a(2)*time + a(3)*time.^2 + a(4)*time.^3;
    ydot_c(t_c_indices(k):t_c_indices(k+1)) = 0 + a(2) + 2*a(3)*time + 3*a(4)*time.^2;
end

end

function T = polynomial_matrix(t0, t1)
    T = [
            [1, t0, t0^2, t0^3],
            [1, t1, t1^2, t1^3],
            [0, 1, 2*t0, 3*t0^2],
            [0, 1, 2*t1, 3*t1^2]
        ];
end