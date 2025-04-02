function [Ai, bi] = calculate_thruster_inequality_matrix(N, z_dim, u0, max_inputs, max_delta_u)
% This function computes the inequality matrice and vector used for
% limiting thruster input

% Ensure that the max_inputs, max_delta_u and u0 values are stored in a column vector
if isrow(max_inputs)
    max_inputs = max_inputs';
end
if isrow(max_delta_u)
    max_delta_u = max_delta_u';
end
if isrow(u0)
    u0 = u0';
end

r_dim = size(max_inputs, 1);

% Compute the max thruster force/momentum limitation
% Ai_upper = [eye(r_dim*N), zeros(r_dim*N, (z_dim-r_dim)*N)];
Ai_upper =[eye(r_dim*N); -eye(r_dim*N)];
bi_upper = repmat(max_inputs, 2*N, 1);

% Compute the max change in force/momentum for each time step
% Ai_lower = [eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1), zeros(r_dim*N, (z_dim-r_dim)*N)];
Ai_lower = [eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1); -(eye(r_dim*N)-diag(ones((r_dim*N-abs(-1)), 1), -1))];
bi_lower = [max_delta_u + u0; repmat(max_delta_u, N-1, 1); max_delta_u - u0; repmat(max_delta_u, N-1, 1)];

% Create inequality constraints
Ai = [Ai_upper; Ai_lower];
bi = [bi_upper; bi_lower];

% Ai = Ai_upper;
% bi = bi_upper;
