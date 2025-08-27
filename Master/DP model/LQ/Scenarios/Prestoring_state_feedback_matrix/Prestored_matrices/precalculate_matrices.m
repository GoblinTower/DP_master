% File that computes the state feedback matrix for different values of psi (heading angle)

addpath("..\..\..\..\..\Tools\");

run '..\dp_model_scenario_LQ_control_stored_matrices_with_disturbance.m'

% Fetch M and D matrices
% See Identification of dynamically positioned ship paper written by Thor
% Inge Fossen et al (1995).
[~, ~, M, D] = supply();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Precalculate matrices %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% State feedback matrix %%%

angle_resolution = 0.1;                     % Angle resoultion in degrees

angles_array = 0:angle_resolution:360;      % Angles array
n_angles = size(angles_array,2);            % Number of angles

G1_dict = dictionary;
G2_dict = dictionary;

for i=1:n_angles

    angle_rad = deg2rad(angles_array(i));

    [A_lin, B_lin, C_lin] = supply_discrete_matrices(M, D, angle_rad, dt, false);

    [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P);

    % Store data
    G1_dict(num2str(angles_array(i),'%.1f')) = {G1};
    G2_dict(num2str(angles_array(i),'%.1f')) = {G2};
end

%%% Kalman gain %%%

Akal = dictionary;
Bkal = dictionary;
Fkal = dictionary;
Ckal = dictionary;
Kkal = dictionary;

for i=1:n_angles

    angle_rad = deg2rad(angles_array(i));

    [A_lin, B_lin, F_lin, C_lin] = dp_fossen_discrete_matrices(M, D, angle_rad, dt, false);

    [K,~,~,~] = dlqe(A_lin, G_lin, C_lin, W, V);

    % Store data
    Akal(num2str(angles_array(i),'%.1f')) = {A_lin};
    Bkal(num2str(angles_array(i),'%.1f')) = {B_lin};
    Fkal(num2str(angles_array(i),'%.1f')) = {F_lin};
    Ckal(num2str(angles_array(i),'%.1f')) = {C_lin};
    Kkal(num2str(angles_array(i),'%.1f')) = {K};

end

% Save data
save("prestored_matrices.mat", "G1_dict", "G2_dict", "Akal", "Bkal", "Fkal", "Ckal", "Kkal");
