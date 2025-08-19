function [A_lin, B_lin, C_lin] = body_model_discrete_matrices(M, D, dt, use_fe)
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
% C_lin              : Discrete linear output matrix
%

% Transition matrix
Ac = [
        zeros(3), eye(3);
        zeros(3), -inv(M)*D
     ];

% Input matrix
Bc = [zeros(3); inv(M)];

% Output matrix
Cc = [eye(3), zeros(3)];

if (use_fe)
    % Calculate discrete matrices using Forward Euler
    A_lin = eye(9) + dt*Ac;
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