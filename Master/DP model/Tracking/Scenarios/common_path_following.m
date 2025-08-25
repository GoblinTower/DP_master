% File containing the the waypoints used in path following (simple tracking)

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Waypoint following %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
distance_to_update_setpoint = 5;
angle_to_update_setpoint = deg2rad(2);

waypoints = [20, 80, 100, 160, 200, 240, 300, 380, 400, 460; 
             10, 60, 100, 60, 120, 180, 200, 300, 240, 260];

waypoints = calculate_angles_waypoints(waypoints, x0(1:2));

last_waypoint_index = size(waypoints, 2);

show_path_plot = false;
if (show_path_plot)
    figure(1);
    hold on;
    plot(waypoints(2,:), waypoints(1,:), 'bo-');
    title("waypoints");
    xlabel("East");
    ylabel("North");
    legend({'Waypoints'}, 'Location', 'Best');
    grid();
    hold off;
end