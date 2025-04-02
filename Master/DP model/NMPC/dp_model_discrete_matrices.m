function [Ad, Bd, Cd, Ad_dot, Bd_dot, Cd_dot] = dp_model_discrete_matrices(x_op, u_op, dt, M, D)

% Ensure operational points are on column vector form
if isrow(x_op)
    x_op = x_op';
end
if isrow(u_op)
    u_op = u_op';
end

% state variables
syms x y psi_angle u v r b1 b2 b3;

% control inputs
syms X Y N;

% Create rotation matrix between NED and vessel coordinates
R = [cos(psi_angle), -sin(psi_angle), 0; sin(psi_angle), cos(psi_angle) 0; 0, 0, 1];

% Solve for R

% Calulcating A matrix
A_symbolic = [
                zeros(3,3), R, zeros(3,3); 
                zeros(3,3), -inv(M)*D, -inv(M)*R;
                zeros(3,3), zeros(3,3), zeros(3,3)
             ];

% Calculating B matrix
B_symbolic = [zeros(3,3); inv(M); zeros(3,3)];

% Need to alter this into a discrete model using forward euler
% approximation
Ad_symbolic = eye(9) + dt*A_symbolic;
Bd_symbolic = dt*B_symbolic;

% Calculate the jacobians
Ad_dot_symbolic = jacobian(Ad_symbolic*[x; y; psi_angle; u; v; r; b1; b2; b3], [x; y; psi_angle; u; v; r; b1; b2; b3]);
Bd_dot_symbolic = jacobian(Bd_symbolic*[X; Y; N], [X; Y; N]);

Ad_numerical = double(subs(Ad_symbolic, [x; y; psi_angle; u; v; r; b1; b2; b3], x_op));
Bd_numerical = double(subs(Bd_symbolic, [X; Y; N], u_op));

Ad_dot_numerical = double(subs(Ad_dot_symbolic, [x; y; psi_angle; u; v; r; b1; b2; b3], x_op));
Bd_dot_numerical = double(subs(Bd_dot_symbolic, [X; Y; N], u_op));

% Return matrices
Ad = Ad_numerical;
Bd = Bd_numerical;
Cd = [eye(3), zeros(3,3), zeros(3,3)];

Ad_dot = Ad_dot_numerical;
Bd_dot = Bd_dot_numerical;
Cd_dot = Cd;