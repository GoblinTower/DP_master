function [A_lin, B_lin, C_lin] = supply_discrete_matrices_si(A_si, B_si, C_si, psi, dt)
% Function calculates model matrices based on identified body dynamics defined
% by A_si, B_si and C_si. These are then transformed into a state space
% model of order 9.
%
% INPUT:
% M                  : Inertia matrix
% D                  : Hydrodynamic damping matrix
% psi                : Vessel heading
% dt                 : Timestep
%
% OUTPUT:
% A_lin              : Discrete linearized state transition matrix
% B_lin              : Discrete linear input matrix
% C_lin              : Discrete linear output matrix
%

% Rotation matrix
rotation = rotation_matrix(psi);

% Transition matrix
A_lin = [
            eye(3), zeros(3,3), dt*rotation;
            zeros(3,3), A_si, zeros(3,3);
            zeros(3,3), C_si*A_si, zeros(3,3);
        ];

% Input matrix
B_lin = [zeros(3,3); B_si; C_si*B_si];

% Output matrix
C_lin = [eye(3), zeros(3,3), zeros(3,3)];

end