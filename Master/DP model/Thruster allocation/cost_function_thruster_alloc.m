function J = cost_function_thruster_alloc(z, alpha0, P, W, Q, Omega, rho, epsilon)
    
    r_dim = size(P, 1);
    
    f = z(1:3);
    alpha = z(4);
    s = z(5:7);

    alpha_diff = alpha - alpha0;

    power_consumption = 0;
    for i=1:r_dim
        power_consumption = power_consumption + P(i)*abs(f(i))^(3/2);
    end
    
    T = thruster_configuration_matrix(alpha);
        
    J = power_consumption + s'*Q*s + alpha_diff'*Omega*alpha_diff + ...
        rho/(epsilon + det(T*inv(W)*T'));

end