function [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si_identity(A_si, B_si, C_si, psi, dt)
% Function calculates model matrices based on identified body dynamics defined
% by A_si, B_si and C_si. These are then transformed into a state space
% model of order n_dim + m_dim + 3.
%
% INPUT:
% A_si               : Discrete state transition matrix for body dynamics
% B_si               : Discrete input matrix for body dynamics
% C_si               : Discrete output matrix for body dynamics
% psi                : Current vessel heading
% dt                 : Timestep
%
% OUTPUT:
% A_lin              : Discrete linearized state transition matrix
% B_lin              : Discrete linear input matrix
% C_lin              : Discrete linear output matrix
%

% Dimensions
n_dim = size(A_si,1);
m_dim = size(C_si,1);
r_dim = size(B_si,2);

% Rotation matrix
rotation = rotation_matrix(psi);

% Transition matrix
A_lin = [
            eye(3), dt*rotation;
            zeros(n_dim,3), A_si
        ];

% Input matrix
B_lin = [zeros(3,r_dim); B_si];

% Output matrix
C_lin = [eye(3), zeros(3,n_dim)];

end