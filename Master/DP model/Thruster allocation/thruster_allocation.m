% Script for thruster allocation
% Ship is described in configuration file

% Define thruster configuration matrix
% First column: Main propeller port
% Second column: Main propeller starboard
% Third column: Bow tunnel
main_propeller_y_distance = 3;
thruster_x_distance = 30;
T_conf = [1, 1, 0;
          0, 0, 1;
          main_propeller_y_distance, -main_propeller_y_distance, thruster_x_distance];

% Define force coefficient matrix
propeller_port_force_coefficient = 1000;
propeller_starboard_force_coefficient = propeller_port_force_coefficient;
bow_thruster_force_coefficient = 500;
K_force = diag(propeller_port_force_coefficient, propeller_starboard_force_coefficient, ...
    bow_thruster_force_coefficient);

% RPM or pitch angle required are given in u vector.
% Assume here that force proportional to the square of the RPM or pitch
% angle values. Alternatively it can be a linear relation.
% We now have:
%
% f = [X; Y; N] = T_conf x K_force x u^
%
% u^ = u                            when linear relation between RPM / Pitch angle
% u = sign(u^) x sqrt(|u^|)         when quadratic relation between RPM / Pitch angle

% Solving using Lagrange Multiplier, unconstrained
W = eye(3);                         % Weighting matrix
T = inv(W)*T_conf'*inv(T_conf*inv(W)*T_conf');

