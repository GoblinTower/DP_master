function [c, ceq] = non_linear_constraints_alloc(z, tau, alpha0, fmin, fmax, alpha_min, alpha_max, alpha_diff_min, alpha_diff_max, thr_conf_mat)
% Simple function for calculating non-linear constraints when running
% thruster allocation for setup that includes azimuth thruster.
% This function is used together with the cost function named
% 'cost_function_thruster_alloc.m'.
%
% INPUTS:
% z               : Optimization variable: z = [f1, f2, ... fn, a1, a2, ... am, s1, s2, s3].
%                   The fi variables represents individual propulsive devices (thrusters,
%                   propellers and azimuths), ai represents the azimuth.
%                   angles and s1, s2 and s3 are slack variables. 
% tau             : tau is the force demand from the controller: 
%                   tau = [force north, force east, momentum yaw]
% alpha0          : Angle of azimuths in previous timestep.                  
% fmin            : Minimum force limitation due to thruster saturation.
% fmin            : Maximum force limitation due to thruster saturation.
% alpha_min       : Minimum azimuth angle.
% alpha_max       : Maximum azimuth angle.
% alpha_diff_min  : Minimum azimuth angle difference (alpha_k = alpha_k - alpha_{k-1}) limitation.
% alpha_diff_max  : Maximum azimuth angle difference (alpha_k = alpha_k - alpha_{k-1}) limitation.
% thr_conf_mat    : Function handle to function that calculates the thruster configuration matrix. 
%                   Takes the alpha (azimuth angles) as input - thr_conf_mat(alpha).
%
% OUTPUTS:
% c               : Inequality constraints: c*z <= 0
% ceq             : Equality constraints: ceq*z = 0
%

% Dimensions
n_thrusters = length(fmin);       % Number of forces
n_azimuths = length(alpha_min);   % Number of azimuths
n_slack_variables = 3;            % Number of slack variables

% Unravel optimization variable
f = z(1:n_thrusters);                                                               % Thruster force
alpha = z(n_thrusters+1:n_thrusters+n_azimuths);                                    % Azimuth angles
s = z(n_thrusters+n_azimuths+1:n_thrusters+n_azimuths+n_slack_variables);           % Slack variables

% Get thruster configuration matrix
T = thr_conf_mat(alpha);

% Equality constraints
ceq = T*f - tau - s;

% Inequality constraints
c = zeros(2*n_thrusters+4*n_azimuths,1);                                                       % Allocate memory

c(1:n_thrusters) = f - fmax;                                                                   % Maximum thruster force constraint
c(n_thrusters+1:2*n_thrusters) = fmin - f;                                                     % Minimum thruster force constraint

c(2*n_thrusters+1:2*n_thrusters+n_azimuths) = alpha - alpha_max;                               % Maximum azimuth angle force constraint
c(2*n_thrusters+n_azimuths+1:2*n_thrusters+2*n_azimuths) = alpha_min - alpha;                  % Minimum azimuth angle force constraint

alpha_diff = alpha - alpha0;
c(2*n_thrusters+2*n_azimuths+1:2*n_thrusters+3*n_azimuths) = alpha_diff - alpha_diff_max;      % Maximum azimuth angle change constraint
c(2*n_thrusters+3*n_azimuths+1:2*n_thrusters+4*n_azimuths) = alpha_diff_min - alpha_diff;      % Minimum azimuth angle change constraint

end
