function x_dot = supply_model(t, x, u, M, D, wind, current)
    
    % Input matrix
    Bc = [zeros(3,3); inv(M)];
            
    % Non-linear state transition matrix
    R = rotation_matrix(x(3));
    Ac = [
        zeros(3,3), R; 
        zeros(3,3), -inv(M)*D,
    ];

    x_dot = Ac*x + Bc*u + Bc*(wind + current);

end