function J = integrator_identification_objective_function_mode1(b, y, u, W)
% Assuming the underlying state space model is an integrator. This implies that
%
% x_kp1 = A*x_k + Bu                 (eq. 1)
% y = C*x                            (eq. 2)
%
% Where it is assumed that,
%
%     | 1   0   0 |            | 1   0   0 |
% A = | 0   1   0 |        C = | 0   1   0 |
%     | 0   0   1 |            | 0   0   1 |
%
% It is assumed that the initial value is x(t0) = [0 0 0]'
%
% The goal is to identify the form of B, i.e. find the entries in B 
% (b11, b12, b13, b21 ... b33) such that the prediction error is minimized.
%
%     | b11  b12  b12 |
% B = | b21  b22  b23 |
%     | b31  b32  b33 |
%   
% The prediction error crieria to be minimized is
% 
% J = Sum_i (e_k'*I*ek)
%
% Where I is a 3x3 identitiy matrix and e_k is the prediction error as 
% given below.
%
% e_k = y_k_real - y_k_estimated
%
% INPUT:
% b                  : Estimated entries of input matrix, 
%                      b = [b11, b12, b13, b21 ... b33]
% y                  : Process output (real data)
% u                  : Input data (real data)
%
% OUTPUT:
% J                  : Cost function value
%

% Initialize values
x0 = [0; 0; 0];                     % Initial state
J = 0;                              % Cost function value

% Known state space matrices
A = eye(3);                         % State transition matrix
C = eye(3);                         % Output matrix

% Uknown state space matrices
B = reshape(b,[3,3])';              % Input matrix

% Number of samples 
n_samples = size(y,2);

% Estimate output from state space model
x = x0;
for k=1:n_samples

    % Caculate output variables
    y_est = C*x;

    prediction_error = y(:,k) - y_est;

    % Calculate cost function
    J = J + prediction_error'*W*prediction_error;

    x = A*x + B*u(:,k);

end

end
