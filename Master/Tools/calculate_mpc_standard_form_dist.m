function [H, c, Ae, be] = calculate_mpc_standard_form_dist(P, Q, A, B, C, F, tau, x0, N, ref)
% This function computes the matrices of MPC on standard form:
%
% (1/2)*z'*H*z + c'z
%
%   Subject to:
%   Ae*z = be
%   Ai*z <= bi
%   zL <= z <= zU
%
% Assumes A to be constant

% Ensure that the reference values are stored in a column vector
if isrow(ref)
    ref = ref';
end

n_dim = size(A, 1);
m_dim = size(C, 1);
r_dim = size(B, 2);
z_dim = N*(r_dim + n_dim + 2*m_dim);

% Compute H matrix
%
%     |H11  0   0   0 |
% H = | 0  H22  0   0 |
%     | 0   0  H33  0 |
%     | 0   0   0  H44|

H11 = kron(eye(N), P);
H22 = kron(eye(N), zeros(n_dim, n_dim));
H33 = kron(eye(N), Q);
H44 = kron(eye(N), zeros(m_dim, m_dim));

H = blkdiag(H11, H22, H33, H44);

% Compute c vector
c = zeros(z_dim, 1);

% Compute A_e matrix (equality constraint)
%
%      |Ae1u Ae1x Ae1e Ae1y|
% Ae = |Ae2u Ae2x Ae2e Ae2y|
%      |Ae3u Ae3x Ae3e Ae3y|
%
%      |be1|
% be = |be2|
%      |be3|
%
% Related by Ae*z = be

% First row
Ae1u = -kron(eye(N), B);
Ae1x = eye(N*n_dim) - kron(diag(ones(N-abs(-1),1),-1), A);
Ae1e = zeros(N*n_dim, N*m_dim);
Ae1y = zeros(N*n_dim, N*m_dim);
be1 = [A*x0 + F*tau; kron(ones(N-1,1), F*tau)];

% Second row
Ae2u = zeros(N*m_dim, N*r_dim);
Ae2x = -kron(eye(N), C);
Ae2e = zeros(N*m_dim, N*m_dim);
Ae2y = eye(N*m_dim);
be2 = zeros(N*m_dim, 1);

% Third row
Ae3u = zeros(N*m_dim, N*r_dim);
Ae3x = zeros(N*m_dim, N*n_dim);
Ae3e = eye(N*m_dim);
Ae3y = eye(N*m_dim);
be3 = ref;

Ae = [
        Ae1u, Ae1x, Ae1e, Ae1y;
        Ae2u, Ae2x, Ae2e, Ae2y;
        Ae3u, Ae3x, Ae3e, Ae3y
     ];

be = [be1; be2; be3];
