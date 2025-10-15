% Script for fetching model matrices in SI model
clear, clc, close all;

addpath("..\..\Tools\");

% Get matrices
% model = 'supply';
% model = 'osv';
model = 'balchen';

identification_name_strings = {'dsr', 'dsr_e', 'pem'};

for i=1:3

    % Path to workspace
    path = strcat('Log\', cell2mat(identification_name_strings(i)), '_ssm_', model);

    % Load matrices
    si = load(path);
    A_si = si.A;
    B_si = si.B;
    C_si = si.C;
    
    dt = si.dt;

    % We do not care about the upper right corner of the matrix
    psi = 0;
    
    [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si(A_si, B_si, C_si, psi, dt);

    if (strcmp(cell2mat(identification_name_strings(i)), 'dsr' ))
        dsr_A = A_lin;
        dsr_B = B_lin;
        dsr_C = C_lin;
    elseif (strcmp(cell2mat(identification_name_strings(i)), 'dsr_e' ))
        dsr_e_A = A_lin;
        dsr_e_B = B_lin;
        dsr_e_C = C_lin;
    elseif (strcmp(cell2mat(identification_name_strings(i)), 'pem' ))
        pem_A = A_lin;
        pem_B = B_lin;
        pem_C = C_lin;
    end
    
    disp(cell2mat(identification_name_strings(i)));
    disp('A:');
    disp(A_lin);
    disp('B:');
    disp(B_lin);
    % disp('C:');
    % disp(C_lin);

end

% Check that DSR and DSR_e gives same results
disp('Diff A (DSR vs DRS_e)');
disp(dsr_A - dsr_e_A);
disp('Frobenius diff A (DSR vs DRS_e)');
disp(norm(dsr_A - dsr_e_A))
disp('Diff B (DSR vs DRS_e)');
disp(dsr_B - dsr_e_B);
disp('Frobenius diff B (DSR vs DRS_e)');
disp(norm(dsr_B - dsr_e_B))
% disp('Diff C (DSR vs DRS_e)');
% disp(dsr_C - dsr_e_C);

% Check difference between DSR and PEM
disp('Diff A (DSR vs PEM)');
disp(dsr_A - pem_A);
disp('Frobenius diff A (DSR vs PEM)');
disp(norm(dsr_A - pem_A))
disp('Diff B (DSR vs PEM)');
disp(dsr_B - pem_B);
disp('Frobenius diff B (DSR vs PEM)');
disp(norm(dsr_B - pem_B))
% disp('Diff C (DSR vs PEM)');
% disp(dsr_C - pem_C);
