% Scripts to investigate the effect of linearizing the model
% using Taylor series approximation.
clear all, close all, clc;

% Operational points
x_op = [0; 0; 0; 0; 0; 0];
u_op = [0; 0; 0];

n_states = size(x_op, 1);
degree_step = 1.0;

% Calculate eigenvalues for every integer degree
% All other states remains in zero position
degree_array = 0:degree_step:(360);
n_degrees = length(degree_array);

transition_matrix_array = zeros(n_states, n_states, n_degrees);
for k=1:n_degrees

    % Change the operational yaw position
    degree = degree_array(k);
    psi = deg2rad(degree);
    x_op(3) = psi;

    % Fetch linearized matrices
    [A, ~, ~] = linearizing_model(x_op, u_op);

    transition_matrix_array(:,:,k) = A;
end

% Calculate change in matrices using Frobenius norm and eigenvalues
frobenius = zeros(1, n_degrees);
frobenius_difference = zeros(1, n_degrees-1);
eigenvalues = zeros(n_states, n_degrees);
for k=1:n_degrees
    frobenius(k) = norm(transition_matrix_array(:,:,k));
    if (k ~= n_degrees)
        frobenius_difference(k) = norm(transition_matrix_array(:,:,k+1) - transition_matrix_array(:,:,k), 'fro');
    end
    % Important to sort since the eigenvalue order is not 
    % guaranteed by MATLAB
    eigenvalues(:,k) = sort(eig(transition_matrix_array(:,:,k)));
end

max_frobenius = max(frobenius);
min_frobenius = min(frobenius);
fprintf('Max and min of frobenius norm is %f and %f\n\n', max_frobenius, min_frobenius);

max_frobenius_change = max(abs(frobenius_difference));
fprintf('Max forbenius change is %f\n\n', max_frobenius_change);

% Plot the frobenius metric
figure(1);
subplot(2,1,1);
plot(degree_array, frobenius);
grid();
title('Frobenius norm of each transition matrix, A(Ψ)');
xlabel('Degree [°]');
ylabel('Frobenius norm');
subplot(2,1,2);
plot(degree_array(1:end-1), frobenius_difference);
grid();
title('Frobenius norm of A(Ψ + 1°) - A(Ψ)');
xlabel('Degree [°]');
ylabel('Frobenius norm');

% Plot eigenvalues
figure(2);
for k=1:6
    subplot(3,2,k);
    colormap('jet');
    scatter(real(eigenvalues(k,:)), imag(eigenvalues(k,:)), 80, degree_array);
    colorbar;
    grid();
    title(['Eigenvalue λ_', num2str(k), ' for different values of yaw (Ψ)']);
    xlabel('Real axis');
    ylabel('Imaginary axis');
end

% Plot real eigenvalues
if isreal(eigenvalues)
    figure(3);
    for k=1:6
        subplot(3,2,k);
        plot(degree_array, real(eigenvalues(k,:)));
        grid();
        title(['Eigenvalue λ_', num2str(k), ' for different values of yaw (Ψ)']);
        xlabel('Degree [°]');
        ylabel('Eigenvalue');
    end
else
    fprintf('Some of the eigenvalues have an imaginary component');
end
