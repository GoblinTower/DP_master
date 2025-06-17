function [Ai, bi] = calculate_force_inequality_matrix(N, u_m1, n_dim, m_dim, r_dim, max_inputs, max_delta_u)
% This function computes the inequality matrice and vector used for
% limiting force input in MPC formulation

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
Ai_upper =[eye(r_dim*N), zeros(r_dim*N,(n_dim + 2*m_dim)*N)]; %; -eye(r_dim*N), zeros(r_dim*N,(n_dim + 2*m_dim)*N)];
bi_upper = repmat(max_inputs, N, 1);

% Compute the max change in force/momentum for each time step
% Ai_lower = [eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1), zeros(r_dim*N,(n_dim + 2*m_dim)*N); ...
%     -(eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1)), zeros(r_dim*N,(n_dim + 2*m_dim)*N)];
% bi_lower = [max_delta_u + u_m1; repmat(max_delta_u, N-1, 1); max_delta_u - u_m1; repmat(max_delta_u, N-1, 1)];

% Create inequality constraints
% Ai = [Ai_upper; Ai_lower];
% bi = [bi_upper; bi_lower];

Ai = [Ai_upper];
bi = [bi_upper];
