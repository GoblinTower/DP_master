function [At, Bt, Gt, Ct] = include_integrator_ssm_with_specified_noise(A, B, G, C)
% Includes an integrator in the discrete SSM model.
% This is necessary to deal with colored noise in the state equation.
% 
% This is an extension of the include_integrator_ssm() function,
% that allows for specifying noise for only a select few state variables.
% Note that inserting G = I gives same results as running
% include_integrator_ssm()
%
% The original equation takes the following form:
%
% x(k+1) = A*x(k) + B*u(k) + G*w(k)       (State equation)
% y(k) = C*x(k) + v(k)                    (Output equation)
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
% At = [A, G; 0, I]
% Bt = [B; 0]
% Gt = [0; I]
% Ct = [C, 0]
% xt = [x; w]
% 
% INPUTS:
% A               : State transition matrix. 
% B               : Input matrix.
% G               : Process noise matrix
% C               : Output matrix.
%
% OUTPUTS:
% At              : Augmented state transition matrix.
% Bt              : Augmented input matrix.
% Gt              : Augmented process noise matrix.
% Ct              : Augmented output matrix.
%

% Get dimensions
n_dim = size(A,1);      % Number of states
r_dim = size(B,2);      % Number of inputs
w_dim = size(G,2);      % Number of noise inputs
m_dim = size(C,1);      % Number of measurements

At = [A, G; zeros(w_dim, n_dim), eye(w_dim)];        % Augmented state transition matrix
Bt = [B; zeros(w_dim, r_dim)];                       % Augmented input matrix
Gt = [zeros(n_dim, w_dim); eye(w_dim)];              % Augmented process noise matrix
Ct = [C, zeros(m_dim, w_dim)];                       % Augmented output matrix

end