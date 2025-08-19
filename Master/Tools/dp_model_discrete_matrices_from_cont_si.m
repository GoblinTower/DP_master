function [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_from_cont_si(Ac_si, Bc_si, Cc_si, psi, dt)
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

% Rotation matrix
rotation = rotation_matrix(psi);

% Transition matrix
Ac = [
            zeros(3,3), zeros(3,3), rotation;
            zeros(3,3), Ac_si, zeros(3,3);
            zeros(3,3), Cc_si*Ac_si, zeros(3,3)
     ];

% Input matrix
Bc = [zeros(3,3); Bc_si; Cc_si*Bc_si];

% Output matrix
Cc = [eye(3), zeros(3,3), zeros(3,3)];

% Convert to discrete model
sysc = ss(Ac, Bc, Cc, zeros(3,3));
sys = c2d(sysc, dt);

% Output
A_lin = sys.A;
B_lin = sys.B; 
C_lin = sys.C;

end