function [A_lin, B_lin, C_lin] = suppy_discrete_matrices(M, D, psi, dt, use_fe)
% Function calculates the discrete matrices for the supply model
% based on the "Identification of dynamically positioned ships" paper 
% written by Tor Inge Fossen et al (1995).
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
Ac = [
    zeros(3,3), rotation; 
    zeros(3,3), -inv(M)*D; 
];

% Input matrix
Bc = [zeros(3,3); inv(M)];

% Output matrix
Cc = [eye(3), zeros(3,3)];

if (use_fe)
    % Calculate discrete matrices using Forward Euler
    A_lin = eye(6) + dt*Ac;
    B_lin = dt*Bc;
    C_lin = Cc;
else
    % Calculate using inbuild ss() and c2d() in MATLAB
    sys = ss(Ac, Bc, Cc, 0);
    sysd = c2d(sys, dt);

    A_lin = sysd.A;
    B_lin = sysd.B;
    C_lin = sysd.C;
end

end