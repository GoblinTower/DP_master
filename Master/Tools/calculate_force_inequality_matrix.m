function [Ai, bi] = calculate_force_inequality_matrix(N, u_m1, n_dim, m_dim, r_dim, max_inputs, max_delta_u)
% This function computes the inequality matrice and vector used for
% limiting force input in MPC formulation
% The optimization variable consists of the following terms:
%
% z = [u, x, e, y, du]          (eq. 1)
%
% Where u is the control input, x is the state variable, e is the error 
% (e = r - y), y is the output and du is the control input difference 
% (du_k = u_k - u_(k-1)).
%
% The optimization variable must hence satisfy the followin condition:
%
% Ai*z <= bi                    (eq. 2)
%
% INPUT:
% N                  : Number of samples in the prediction horizon
% u_m1               : The previous control input, u_(k-1)
% n_dim              : Number of states
% m_dim              : Number of outputs
% r_dim              : Number of inputs
% max_inputs         : Max value of inputs (max_u = [max_su, max_sw, max_yaw])
% max_delta_u        : Max delta u of inputs (max_du = [max_su, max_sw, max_yaw])
%
% OUTPUT:
% Ai                 : Inequality matrix (see eq. 2)
% bi                 : Inequality vector (see eq. 2)
%

% Ensure that the max_inputs, max_delta_u and u0 values are stored in a column vector
if isrow(max_inputs)
    max_inputs = max_inputs';
end
if isrow(max_delta_u)
    max_delta_u = max_delta_u';
end
if isrow(u_m1)
    u_m1 = u_m1';
end

% Add absolute force/momentum limitation
Ai_u =[eye(r_dim*N), zeros(r_dim*N,(n_dim + 2*m_dim + r_dim)*N); -eye(r_dim*N), zeros(r_dim*N,(n_dim + 2*m_dim + r_dim)*N)];
bi_u = repmat(max_inputs, 2*N, 1);

% Compute the max change in force/momentum for each time step
% Ai_lower = [eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1), zeros(r_dim*N,(n_dim + 2*m_dim)*N); ...
%     -(eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1)), zeros(r_dim*N,(n_dim + 2*m_dim)*N)];
% bi_lower = [max_delta_u + u_m1; repmat(max_delta_u, N-1, 1); max_delta_u - u_m1; repmat(max_delta_u, N-1, 1)];

% Create inequality constraints
% Ai = [Ai_upper; Ai_lower];
% bi = [bi_upper; bi_lower];

Ai = [Ai_u];
bi = [bi_u];
