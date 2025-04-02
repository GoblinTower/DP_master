function [A, B, C] = linearizing_model(x_op, u_op)

% Ensure operational points are on column vector form
if isrow(x_op)
    x_op = x_op';
end
if isrow(u_op)
    u_op = u_op';
end

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% A linear model can be constructed from the orignal supply model

% state variables
syms x y psi_angle u v r;

% control inputs
syms X Y N;

% Create rotation matrix between NED and vessel coordinates
R = [cos(psi_angle), -sin(psi_angle), 0; sin(psi_angle), cos(psi_angle) 0; 0, 0, 1];
 
% Calulcating A matrix
A_symbolic = [zeros(3,3), R; zeros(3,3) -inv(M)*D];

% Computing linearized matrix
A_lin_symbolic = jacobian(A_symbolic*[x; y; psi_angle; u; v; r], [x; y; psi_angle; u; v; r]);
A_lin_subs = subs(A_lin_symbolic, [x; y; psi_angle; u; v; r], x_op);
A_num = double(A_lin_subs);

% Calculating B matrix
B = [zeros(3,3); inv(M)];

% Computing linearized matrix
B_lin_symbolic = jacobian(B*[X; Y; N], [X; Y; N]);
B_lin_subs = subs(B_lin_symbolic, [X; Y; N], u_op);
B_num = double(B_lin_subs);

% Return state transition and input matrices
A = A_num;
B = B_num;
C = [eye(3), zeros(3)];
