function [c, ceq] = non_linear_constraints_alloc(z, tau, alpha0, fmin, fmax, alpha_min, alpha_max, alpha_diff_min, alpha_diff_max)
% Simple function for calculating non-linear constraints

% Unravel optimization variable
f = z(1:4);
alpha = z(5:6);
s = z(7:9);

% Get thruster configuration matrix
T = thruster_configuration_matrix(alpha);

% Equality constraints
ceq = T*f - tau - s;

% Inequality constraints
c = zeros(16,1);                             % Allocate memory

c(1:4) = f - fmax;                           % Maximum thruster force constraint
c(5:8) = fmin - f;                           % Minimum thruster force constraint

c(9:10) = alpha - alpha_max;                 % Maximum azimuth angle force constraint
c(11:12) = alpha_min - alpha;                % Minimum azimuth angle force constraint

alpha_diff = alpha - alpha0;
c(13:14) = alpha_diff - alpha_diff_max;      % Maximum azimuth angle change constraint
c(15:16) = alpha_diff_min - alpha_diff;      % Minimum azimuth angle change constraint

end