function J = non_linear_green_dp_objective_function(u, tau, ref, M, D, P, R1, W1, W2, x0, horizon_length, dt, method)
% Calculation of objective function based on specified control input

J = 0;

Cc = [eye(3), zeros(3,3), zeros(3,3)];

% Integrating function
x = x0;
t = 0;
for k=1:horizon_length

    % Runge-Kutta 4th order
    [~, x] = runge_kutta_4(@(t, x) dp_model(t, x, u(:,k), tau, M, D), t, x, dt);

    % Caculate output variables
    y = Cc*x;

    % Calculate input difference
    % if (k == 1)
    %     du = u(:,1) - u0;
    % else
    %     du = u(:,k) - u(:,k-1);
    % end

    % Calculate equclidian distance from reference point
    distance = sqrt((ref(1,k) - y(1))^2 + (ref(2,k) - y(2))^2);
    
    J = J + smooth_cost_function(distance, R1, W1, W2) + u(:,k)'*P*u(:,k);

    t = t + dt;
end

end
