function [G, G1, G2, A_dev, B_dev, C_dev] = calculate_lq_deviation_gain(A_lin, B_lin, C_lin, Q, P)
% Function computes the linear quadratic feedback gain for LQ optimal
% control. This is based on the paper "Discrete LQ optimal control with integral action"
% by David de Ruscio
%
% INPUT:
% A_lin              : Linear state transition matrix
% B_lin              : Linear input matrix
% C_lin              : Linear Output matrix
% Q                  : State weighting matrix
% P                  : Input weighting matrix
%
% OUTPUT:
% G                  : Controller Gain
% G1                 : Gain (state)
% G2                 : Gain (output)
% A_dev              : Deviation form of A
% B_dev              : Deviation form of B
% C_dev              : Deviation form of C
%

n_dim = size(A_lin, 1);    % Number of states
m_dim = size(C_lin, 1);    % Number of outputs
r_dim = size(B_lin, 2);    % Number of inputs

% Calculation of deviation matrices
A_dev = [A_lin, zeros(n_dim, m_dim); C_lin, eye(m_dim, m_dim)];
B_dev = [B_lin; zeros(m_dim, r_dim)];
C_dev = [C_lin, eye(m_dim, m_dim)];

Q_dev = C_dev'*Q*C_dev;

% Calculate gain matrix
[G,~,~] = dlqr(A_dev, B_dev, Q_dev, P);
G1 = -G(:,1:n_dim);
G2 = -G(:,n_dim+1:n_dim+m_dim);

end