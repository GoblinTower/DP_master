function x_dot = dp_model(t, x, u, tau, M, D)
% This model is based on the linear time-varying model DP model defined
% by Fossen on page 181 in his book "Handbook of Marine Craft
% Hydrodynamics and Motion Control, Second Edition".

% Input matrix
Bc = [zeros(3,3); inv(M); zeros(3,3)];
        
% Non-linear state transition matrix
rotation = rotation_matrix(x(3));
Ac = [
    zeros(3,3), rotation, zeros(3,3); 
    zeros(3,3), -inv(M)*D, inv(M)*rotation'; 
    zeros(3,3), zeros(3,3), zeros(3,3)
];

x_dot = Ac*x + Bc*u + Bc*tau;

end