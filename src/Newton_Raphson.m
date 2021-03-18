function [x] = Newton_Raphson(fun,grad,x0,maxiter)
%NEWTON_RAPHSON finds the zeros of a function given its gradient using the
%Newton Raphson root finding algorithm.
%
%   [x, fval] = NEWTON_RAPHSON(fun, grad, x0, maxiter) performs the NR
%   algorithm with the function fun and the jacobian grad, starting at x0
%   and performing maxiter iterations.

% Initialize
x = x0;

PLOT__PP = true;
if PLOT__PP
    figure;
    hold all
end
% Begin algorithm
for ii = 1 : maxiter

    % Calculate function at current input
    Fx = fun(x);
    
    % Calculate jacobian at current input
    Jx = grad(x);
    
    % Perform update
    x = x - pinv(Jx) * Fx;
    
    if PLOT__PP
        stem(ii, Fx);
    end
end

end

