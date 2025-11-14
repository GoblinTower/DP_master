% This scripts outputs the symbolic formulas for A and B and
% calculates the eigenvalues in symbolic form
clear all, close all, clc;

addpath("..\..\Tools\");

% strategy 1
load_path = 'Log\integrator_ssm_supply_mode1';

si = load(load_path);
A_si = si.A;
B_si = si.B;
C_si = si.C;
dt = si.dt;

syms psi
rotation = rotation_matrix(psi);

[A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si_identity(A_si, B_si, C_si, psi, dt);

disp('Eigenvalues, strategy 1');
eigenvalues = eig(A_lin);
disp(eigenvalues);

clear all;

% strategy 2
load_path = 'Log\integrator_ssm_supply_mode4';

si = load(load_path);
A_si = si.A;
B_si = si.B;
C_si = si.C;
dt = si.dt;

syms psi
rotation = rotation_matrix(psi);

[A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si_identity(A_si, B_si, C_si, psi, dt);

disp('Eigenvalues, strategy 2');
eigenvalues = eig(A_lin);
disp(eigenvalues);

clear all;

% strategy 3
load_path = 'Log\integrator_ssm_supply_mode2';

si = load(load_path);
A_si = si.A;
B_si = si.B;
C_si = si.C;
dt = si.dt;

syms psi
rotation = rotation_matrix(psi);

[A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si(A_si, B_si, C_si, psi, dt);

disp('Eigenvalues, strategy 3');
eigenvalues = eig(A_lin);
disp(eigenvalues);

clear all;
