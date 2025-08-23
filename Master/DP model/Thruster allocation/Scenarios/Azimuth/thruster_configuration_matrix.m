function T = thruster_configuration_matrix(alpha)
% Define thruster configuration matrix
% First column: Main propeller port
% Second column: Main propeller starboard
% Third column: Bow azimuth
% Fourth column: Aft azimuth

% Thruster setup
main_port_propeller_y_distance = -3;
main_starboard_propeller_y_distance = 3;

azimuth1_x_distance = 30;
alpha1 = alpha(1);

azimuth2_x_distance = -20;
alpha2 = alpha(2);

T = [
        1, 1, cos(alpha1), cos(alpha2);
        0, 0, sin(alpha1), sin(alpha2);
        -main_port_propeller_y_distance, -main_starboard_propeller_y_distance, azimuth1_x_distance*sin(alpha1), azimuth2_x_distance*sin(alpha2)
    ];
end