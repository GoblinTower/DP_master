function [H, c, Ae, be] = calculate_lpv_mpc_delta_u_dist(P, Q, A, B, C, F, tau, x0, u, N, ref, dt, M, D)
% This function computes the matrices of MPC on standard form:
%
% (1/2)*z'*H*z + c'z
%
%   Subject to:
%   Ae*z = be
%   Ai*z <= bi
%   zL <= z <= zU
%
% Assumes A is calculated based on yaw angle obtained by using present
% state vector and contro signal calculated in previous MPC iteration.

% Ensure that the reference values are stored in a column vector
if isrow(ref)
    ref = ref';
end

n_dim = size(A, 1);
m_dim = size(B, 2);
r_dim = size(C, 1);
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

% Must recompute A for every timestep
Ae1x = eye(N*n_dim);

% First row
Ae1u = -kron(eye(N), B);

x = x0;

%% First row if Ae

% State transition matrix is updated at every time step due to changing yaw angle
for i=1:(N-1)

    % Get yaw angle
    psi = x(3);

    % Get control signal from previous MPC iteration
    if (i == N-1)
        u_signal = u(:,i);
    else
        u_signal = u(:,i+1);
    end

    % Get discrete DP model matrices
    [Ad, Bd, ~, ~] = dp_fossen_discrete_matrices(M, D, psi, dt, false);

    % Estimate future state vectors
    x = Ad*x + Bd*u_signal;

    Ae1x((i*n_dim+1):((i+1)*n_dim),((i-1)*n_dim+1):(i*n_dim)) = -Ad;
end

Ae1e = zeros(N*n_dim, N*m_dim);
Ae1y = zeros(N*n_dim, N*m_dim);
Ae1du = zeros(N*n_dim, N*r_dim);
be1 = [A*x0 + F*tau; kron(ones(N-1,1), F*tau)];

%% Second row
Ae2u = zeros(N*m_dim, N*r_dim);
Ae2x = -kron(eye(N), C);
Ae2e = zeros(N*m_dim, N*m_dim);
Ae2y = eye(N*m_dim);
Ae2du = zeros(N*m_dim, N*r_dim);
be2 = zeros(N*m_dim, 1);

%% Third row
Ae3u = zeros(N*m_dim, N*r_dim);
Ae3x = zeros(N*m_dim, N*n_dim);
Ae3e = eye(N*m_dim);
Ae3y = eye(N*m_dim);
Ae3du = zeros(N*m_dim, N*r_dim);
be3 = ref;

%% Fourth row
Ae4u = eye(N*r_dim) - kron(diag(ones(N-abs(-1),1),-1), eye(r_dim));
Ae4x = zeros(N*r_dim, N*n_dim);
Ae4e = zeros(N*r_dim, N*m_dim);
Ae4y = zeros(N*r_dim, N*m_dim);
Ae4du = -eye(N*r_dim);
be4 = [u(:,1); zeros((N-1)*r_dim,1)];

Ae = [
        Ae1u, Ae1x, Ae1e, Ae1y, Ae1du;
        Ae2u, Ae2x, Ae2e, Ae2y, Ae2du;
        Ae3u, Ae3x, Ae3e, Ae3y, Ae3du;
        Ae4u, Ae4x, Ae4e, Ae4y, Ae4du;
     ];

be = [be1; be2; be3; be4];

