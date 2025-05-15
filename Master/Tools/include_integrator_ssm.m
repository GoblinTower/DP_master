function [At, Bt, Gt, Ct] = include_integrator_ssm(A, B, C)
% Includes an integrator in the discrete SSM model.
% This is necessary to deal with colored noise in the state equation.
% The original equation takes the following form:
%
% x(k+1) = A*x(k) + B*u(k) + w(k)       (State equation)
% y(k) = C*x(k) + v(k)                  (Output equation)
%
% The colored noise is removed by adding the following equation
% w(k+1) = w(k) + dw, where dw is white noise.
% 
% The new augmented SSM model then becomes:
%
% xt(k+1) = At*xt(k) + Bt*u(k) + Gt*dw(k)
% y(k) = Ct*xt(k) + v(k)
% 
% Here,
% At = [A, I; 0, I]
% Bt = [B; 0]
% Gt = [0; I]
% Ct = [C, 0]
% xt = [x; w]
% 
% INPUTS:
% A               : State transition matrix. 
% B               : Input matrix.
% C               : Output matrix.
%
% OUTPUTS:
% At              : Augmented state transition matrix.
% Bt              : Augmented input matrix.
% Gt              : Augmented process noise matrix.
% Ct              : Augmented output matrix.
%

% Get dimensions
n_dim = size(A,1);
r_dim = size(B,2);
m_dim = size(C,1);

At = [A, eye(n_dim); zeros(n_dim), eye(n_dim)];      % Augmented state transition matrix
Bt = [B; zeros(n_dim,r_dim)];                        % Augmented input matrix
Gt = [zeros(n_dim); eye(n_dim)];                     % Augmented process noise matrix
Ct = [C, zeros(m_dim, n_dim)];                       % Augmented output matrix

end