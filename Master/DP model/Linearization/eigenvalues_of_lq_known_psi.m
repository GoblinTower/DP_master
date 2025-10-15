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

% Compute eigenvalues
eigenvalues = double(eig(A_symbolic));

fprintf('Eigenvalues of state transition matrix:\n\n');
disp(eigenvalues);

% Double check eigenvalues by manual calculation
syms lambda
m = -inv(M)*D;
c_polynomial = (-lambda)*(-lambda)*(-lambda)*...
    (+(m(1,1)-lambda)*((m(2,2)-lambda)*(m(3,3)-lambda)-(m(2,3)*m(3,2)))...
     -m(2,1)*((m(1,2)*(m(3,3)-lambda))-(m(1,3)*m(3,2)))...
     +m(3,1)*((m(1,2)*m(2,3))-(m(1,3)*(m(2,2)-lambda))));

eigenvalues = double((solve(c_polynomial)));

fprintf('Eigenvalues of state transition matrix (from manual calculations):\n\n');
disp(eigenvalues);

% Complete symbolic solution
syms m11 m12 m13 m21 m22 m23 m31 m32 m33
c_sym_polynomial = (-lambda)*(-lambda)*(-lambda)*...
    (+(m11-lambda)*((m22-lambda)*(m33-lambda)-(m23*m32))...
     -m21*((m12*(m33-lambda))-(m13*m32))...
     +m31*((m12*m23)-(m13*(m22-lambda))));

fprintf('Charateristic equation symbolic:\n\n');
disp(charpoly(c_sym_polynomial, lambda));

% Double check that equation is valid
char_verify_eigenvalues = subs(c_sym_polynomial, [m11, m12, m13, m21, m22, m23, m31, m32, m33], [m(1,1), m(1,2), m(1,3), m(2,1), m(2,2), m(2,3), m(3,1), m(3,2), m(3,3)]);
solve(char_verify_eigenvalues);
disp(double(solve(char_verify_eigenvalues)));
