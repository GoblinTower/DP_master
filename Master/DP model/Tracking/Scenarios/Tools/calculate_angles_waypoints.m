function extended_waypoints = calculate_angles_waypoints(waypoints, x0)
% Function that recieves a set of waypoints. It calculates the 
% angles of each waypoint, i.e. it assumes that the angles at each
% waypoint is pointing along the line between the current and previous
% waypoint.
%
% INPUT:
% waypoints          : 2xn matrix. Each column contains the position of
%                      a waypoint. [x; y] = [north; east].
% x0                 : 3x1 vector. Initial position of ship.
% 
% OUTPUT:
% waypoints          : 3xn matrix. Same matrix as above, but now includes 
%                      a bottom row for waypoint angles.
%

addpath("Tools\");

% Number of waypoints
last_waypoint_index = size(waypoints, 2);

% Calculate angles
angle_array = zeros(1, last_waypoint_index);
for k=1:last_waypoint_index
    if (k == 1)
        angle_array(k) = atan2(waypoints(2,k) - x0(2), waypoints(1,k) - x0(1));
    else
        angle_array(k) = atan2(waypoints(2,k) - waypoints(2,k-1), waypoints(1,k) - waypoints(1,k-1));
    end
end

% Make sure shortest yaw angle change is always selected
angle_array = find_closest_angle(angle_array);

% Return the new extended waypoint array
extended_waypoints = [waypoints; angle_array];

end