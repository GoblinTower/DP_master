function x_dot = supply_model(t, x, u, M, D, wind, wave, current)
% Supply model extended to accept wind, current and wave forces
% Note that the external force inputs are assumed to be described
% in the NED frame. Hence must be converted to body frame
% using the F_body = R(psi)'*F_ned equation.

% Non-linear state transition matrix
rotation = rotation_matrix(x(3));

% Input matrix
Bc = [zeros(3,3); inv(M)];

% Force matrix
Fc = [zeros(3,3); inv(M)*rotation'];
        
Ac = [
    zeros(3,3), rotation; 
    zeros(3,3), -inv(M)*D,
];

x_dot = Ac*x + Bc*u + Fc*(wind + current + wave);

end