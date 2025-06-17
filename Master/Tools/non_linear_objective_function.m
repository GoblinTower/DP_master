function J = non_linear_objective_function(u, tau, ref, M, D, P, Q, x0, horizon_length, dt, method)
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

    J = J + (ref(:,k) - y)'*Q*(ref(:,k) - y) + u(:,k)'*P*u(:,k);

    t = t + dt;
end

end
