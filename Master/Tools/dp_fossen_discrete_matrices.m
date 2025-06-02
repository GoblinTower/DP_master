function [A_lin, B_lin, F_lin, C_lin] = dp_fossen_discrete_matrices(M, D, psi, dt, use_fe)
% Function calculates the discrete matrices for the dp model
% based on the book "Handbook of Marine Craft Hydrodynamics and
% Motion Control, Second Edition" by Thor Inge Fossen.
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
% F_lin              : Discrete linear force input matrix
% C_lin              : Discrete linear output matrix
%

% Rotation matrix
rotation = rotation_matrix(psi);

% Transition matrix
Ac = [
    zeros(3,3), rotation, zeros(3,3); 
    zeros(3,3), -inv(M)*D, inv(M)*rotation';
    zeros(3,3), zeros(3,3), zeros(3,3)
];

% Input matrix
Bc = [zeros(3,3), zeros(3,3); inv(M), inv(M); zeros(3,3), zeros(3,3)];
% Bc = [zeros(3,3); inv(M); zeros(3,3)];

% Output matrix
Cc = [eye(3), zeros(3,3), zeros(3,3)];

if (use_fe)
    % Calculate discrete matrices using Forward Euler
    A_lin = eye(9) + dt*Ac;
    B_lin = dt*Bc(:,1:3);
    F_lin = dt*Bc(:,4:end);
    C_lin = Cc;
 
else
    % Calculate using inbuild ss() and c2d() in MATLAB
    sys = ss(Ac, Bc, Cc, 0);
    sysd = c2d(sys, dt);

    A_lin = sysd.A;
    B_lin = sysd.B(:,1:3);
    F_lin = sysd.B(:,4:end);
    C_lin = sysd.C;
end

end