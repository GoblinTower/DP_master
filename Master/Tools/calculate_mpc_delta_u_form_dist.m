function [H, c, Ae, be] = calculate_mpc_delta_u_form_dist(P, Q, A, B, C, F, tau, x0, um1, N, ref)
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
z_dim = N*(2*r_dim + n_dim + 2*m_dim);

% Compute H matrix
%
%     |H11  0   0   0   0 |
%     | 0  H22  0   0   0 |
% H = | 0   0  H33  0   0 |
%     | 0   0   0  H44  0 |
%     | 0   0   0   0  H55|

H11 = kron(eye(N), zeros(r_dim, r_dim));
H22 = kron(eye(N), zeros(n_dim, n_dim));
H33 = kron(eye(N), Q);
H44 = kron(eye(N), zeros(m_dim, m_dim));
H55 = kron(eye(N), P);

H = blkdiag(H11, H22, H33, H44, H55);

% Compute c vector
c = zeros(z_dim, 1);

% Compute A_e matrix (equality constraint)
%
%      |Ae1u Ae1x Ae1e Ae1y Ae1du|
% Ae = |Ae2u Ae2x Ae2e Ae2y Ae2du|
%      |Ae3u Ae3x Ae3e Ae3y Ae3du|
%      |Ae4u Ae4x Ae4e Ae4y Ae4du|
%
%      |be1|
% be = |be2|
%      |be3|
%      |be4|
%
% Related by Ae*z = be

% First row
Ae1u = -kron(eye(N), B);
Ae1x = eye(N*n_dim) - kron(diag(ones(N-abs(-1),1),-1), A);
Ae1e = zeros(N*n_dim, N*m_dim);
Ae1y = zeros(N*n_dim, N*m_dim);
Ae1du = zeros(N*n_dim, N*r_dim);
be1 = [A*x0 + F*tau; kron(ones(N-1,1), F*tau)];

% Second row
Ae2u = zeros(N*m_dim, N*r_dim);
Ae2x = -kron(eye(N), C);
Ae2e = zeros(N*m_dim, N*m_dim);
Ae2y = eye(N*m_dim);
Ae2du = zeros(N*m_dim, N*r_dim);
be2 = zeros(N*m_dim, 1);

% Third row
Ae3u = zeros(N*m_dim, N*r_dim);
Ae3x = zeros(N*m_dim, N*n_dim);
Ae3e = eye(N*m_dim);
Ae3y = eye(N*m_dim);
Ae3du = zeros(N*m_dim, N*r_dim);
be3 = ref;

% Fourth row
Ae4u = eye(N*r_dim) - kron(diag(ones(N-abs(-1),1),-1), eye(r_dim));
Ae4x = zeros(N*r_dim, N*n_dim);
Ae4e = zeros(N*r_dim, N*m_dim);
Ae4y = zeros(N*r_dim, N*m_dim);
Ae4du = -eye(N*r_dim);
be4 = [um1; zeros((N-1)*r_dim,1)];

Ae = [
        Ae1u, Ae1x, Ae1e, Ae1y, Ae1du;
        Ae2u, Ae2x, Ae2e, Ae2y, Ae2du;
        Ae3u, Ae3x, Ae3e, Ae3y, Ae3du;
        Ae4u, Ae4x, Ae4e, Ae4y, Ae4du;
     ];

be = [be1; be2; be3; be4];
