function [u, f] = unconstrained_nonrotatable_thruster_allocation(W, T, K, tau)
% This function solves the thruster configuration problem where
% the actuators are nonrotatable or such thruster have fixed angle during
% the duration of the thruster allocation prodedure. See the book 'Handbook
% of Marine Craft Hydrodynamics and Motion Control' by Thor Inge Fossen for reference.
%
% The cost function minimized is:
% 
% J = min_f [f'Wf]
%
% Subject to tau - Tf = 0
%
% This problem can be solved analytically without optimization using
% Lagrangian multipliers.
%
% INPUTS:
% W               : Weighting matrix of thruster force outputs f1, f2, ..., fn.
% T               : Thruster configuration matrix, relates tau to individual thruster forces:
%                   tau = Tf                
% K               : Force coefficient matrix, relates thruster control signal to individual thruster forces: 
%                   f = Ku
%                 : The tau values are typically the output from the controller.
%                   tau = [force in surge, force in sway, momentum in yaw]. T
%
% OUTPUTS:
% u               : Control input to thrusters
% f               : Thruster force outputs
% 

% Generalized inverse
R = inv(W)*T'*inv(T*inv(W)*T');

% Individual thruster force output
f = R*tau;

% Control input to thrusters
u = inv(K)*f;
