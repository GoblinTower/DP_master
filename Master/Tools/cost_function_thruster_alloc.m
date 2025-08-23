function J = cost_function_thruster_alloc(z, alpha0, P, W, Q, Omega, rho, epsilon, thr_conf_mat)
% Cost function that solves the thruster allocation problem in the general case
% that includes azimuth thrusters. Based on equations from the book
% Handbook of Marine Craft Hydrodynamics and motion control by Thor Inge
% Fossen.
%
% INPUTS:
% z               : Optimization variable: z = [f1, f2, ... fn, a1, a2, ... am, s1, s2, s3].
%                   The fi variables represents individual propulsive devices (thrusters,
%                   propellers and azimuths), ai represents the azimuth.
%                   angles and s1, s2 and s3 are slack variables. 
% alpha0          : Angle of azimuths in previous timestep.                  
% P               : Power consumption vector defined for each thruster. It is assumed the power 
%                   consumption is related to each thruster by the following equation: 
%                   Power = P(i)*|f|^(3/2)
% W               : Weighting matrix associated with thruster force
%                   (prioritize which thruster to be responsible for generating power).
% Q               : Slack variable weighting matrix.
% Omega           : Azimuth angle change weighting matrix.
% rho             : Scalar weight that sets a balance between maneuverability and power consumption.
% epsilon         : Small number that avoides division by zero.
% thr_conf_mat    : Function handle to function that calculates the thruster configuration matrix. 
%                   Takes the alpha (azimuth angles) as input - thr_conf_mat(alpha).
%
% OUTPUTS:
% J               : Scalar value of cost function
%

% Dimensions
n_thrusters = size(P, 1);         % Number of forces
n_azimuths = size(Omega, 1);      % Number of azimuths
n_slack_variables = size(Q, 1);   % Number of slack variables

assert(n_slack_variables == 3, 'Q is of wrong dimensions, should be 3x3');

% Unravel optimization variable
f = z(1:n_thrusters);                                                               % Thruster force
alpha = z(n_thrusters+1:n_thrusters+n_azimuths);                                    % Azimuth angles
s = z(n_thrusters+n_azimuths+1:n_thrusters+n_azimuths+n_slack_variables);           % Slack variables

alpha_diff = alpha - alpha0;                                                        % Azimuth angle change

% Calculate power consumption
power_consumption = 0;
for i=1:n_thrusters
    power_consumption = power_consumption + P(i)*abs(f(i))^(3/2);
end

% Calculate thruster configuration matrix
T = thr_conf_mat(alpha);
       
% Calculate cost function
J = power_consumption + s'*Q*s + alpha_diff'*Omega*alpha_diff + ...
    rho/(epsilon + det(T*inv(W)*T'));

end