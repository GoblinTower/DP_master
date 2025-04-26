function T = thruster_configuration_matrix(alpha)
% Define thruster configuration matrix
% First column: Main propeller port
% Second column: Main propeller starboard
% Third column: Azimuth propeller

% Thruster setup
main_port_propeller_y_distance = -3;
main_starboard_propeller_y_distance = 3;
azimuth_x_distance = 30;

T = [
        1, 1, cos(alpha);
        0, 0, sin(alpha);
        -main_port_propeller_y_distance, -main_starboard_propeller_y_distance, azimuth_x_distance*sin(alpha)
    ];
end