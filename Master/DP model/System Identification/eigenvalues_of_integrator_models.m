clear all, close all, clc;

addpath("..\..\Tools\");

vessel_names = {'supply', 'osv', 'balchen'};
si_metods = {'dsr', 'dsr_e', 'pem'};

for i=1:3
    for j=1:3

        vessel = cell2mat(vessel_names(i));
        method = cell2mat(si_metods(j));

        % DSR
        load_path = strcat(strcat('Log\', method, '_ssm_', vessel));
        
        si = load(load_path);
        A_si = si.A;
        B_si = si.B;
        C_si = si.C;
        dt = si.dt;
        
        syms psi
        rotation = rotation_matrix(psi);
        
        [A_lin, B_lin, C_lin] = dp_model_discrete_matrices_si(A_si, B_si, C_si, psi, dt);
        
        disp(strcat(['Eigenvalues, ', method, ' ', vessel]));
        eigenvalues = double(eig(A_lin));
        disp(eigenvalues);
        
        clearvars -except vessel_names si_metods i j;
    end
end
