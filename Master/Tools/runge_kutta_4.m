function [tkp1 xkp1] = runge_kutta_4(fun, tk, xk, dt)
% Runge-Kutta Fourth order method for solving differential
% equations. This solver is based on the RK4 algorithm given
% in the book "Programming for Computations - Python, Second Edition"
% by Svein Linge and Hans Petter Langtangen on page 254.
%
% INPUTS:
% fun             : Handle to the function to be solved. 
%                   Must be of the form x_dot = f(t,x).
% tk              : Current timestep
% xk              : Current value of state
% dt              : Sampling interval
%
% OUTPUTS:
% tkp1             : Time of new state value (tk + dt)
% xkp1             : Updated state value at time tk + dt
%

    if (isrow(xk))
        xk = xk';
    end

    f1 = fun(tk, xk);
    f2 = fun(tk + dt/2, xk + dt*f1/2);
    f3 = fun(tk + dt/2, xk + dt*f2/2);
    f4 = fun(tk + dt, xk + dt*f3);

    tkp1 = tk + dt;

    if (isrow(f1))
        xkp1 = xk + (dt/6)*(f1' + 2*f2' + 2*f3' + f4');
    else
        xkp1 = xk + (dt/6)*(f1 + 2*f2 + 2*f3 + f4);
    end
  
end