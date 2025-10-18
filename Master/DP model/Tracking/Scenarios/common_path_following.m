% File containing the the waypoints used in path following (simple tracking)

addpath("Tools\");

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
    
    fig = figure('DefaultAxesFontSize', 20);
    t = tiledlayout(1,2, 'TileSpacing', 'compact');

    nexttile;
    hold on;
    plot(waypoints(2,:), waypoints(1,:), 'bo-');
    title("Waypoints");
    xlabel("East");
    ylabel("North");
    grid();
    axis equal;
    hold off;

    nexttile;
    hold on;
    plot(rad2deg(waypoints(3,:)), 'bo-');
    title("Angles");
    xlabel("Waypoint");
    ylabel("Angle");
    grid();
    hold off;

end