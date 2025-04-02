% This scripts outputs the symbolic formulas for A and B and
% calculates the eigenvalues in symbolic form
clear all, close all, clc;

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by T.I.
% Fossen et al (1995).
[~, ~, M, D] = supply();

% A linear model can be constructed from the orignal supply model

% state variables
syms x y psi u v r;

% control inputs
syms X Y N;

% Create rotation matrix between NED and vessel coordinates
R = [cos(psi), -sin(psi), 0; sin(psi), cos(psi) 0; 0, 0, 1];
 
% Calulcating A matrix
A_symbolic = [zeros(3,3), R; zeros(3,3) -inv(M)*D];

% Computing linearized matrix
A_lin_symbolic = jacobian(A_symbolic*[x; y; psi; u; v; r], [x; y; psi; u; v; r]);
fprintf('State transition matrix (A(Ψ)):\n\n');
disp(A_lin_symbolic);

% Calculating B matrix
B = [zeros(3,3); inv(M)];

% Computing linearized matrix
B_lin_symbolic = jacobian(B*[X; Y; N], [X; Y; N]);
fprintf('Input matrix matrix B:\n\n');
disp(B_lin_symbolic);

% Compute eigenvalues
eigenvalues = double(eig(A_lin_symbolic));

fprintf('Eigenvalues of state transition matrix\n\n');
disp(eigenvalues);

% Double check eigenvalues by manual calculation
syms lambda
m = -inv(M)*D;
c_polynomial = (-lambda)*(-lambda)*(-lambda)*...
    (+(m(1,1)-lambda)*((m(2,2)-lambda)*(m(3,3)-lambda)-(m(2,3)*m(3,2)))...
     -m(2,1)*((m(1,2)*(m(3,3)-lambda))-(m(1,3)*m(3,2)))...
     +m(3,1)*((m(1,2)*m(2,3))-(m(1,3)*(m(2,2)-lambda))));
double(solve(c_polynomial))

fprintf('Eigenvalues of state transition matrix (from manual calculations)\n\n');
disp(eigenvalues);