function f = Bilevel_outer(c, itpParam, modelParam, optParam, liftParam, innerloopParam, outerloopParam)
%BILEVEL_OUTER is an iteration of the outerloop.
%
%   f = BILEVEL_OUTER(c, itpParam, modelParam, optParam, innerloopParam, outerloopParam)

% Set the weights
optParam.CostFunctionWeights = c;

% Optimization inner loop
x_star = ...
        fmincon(...
            @(x)costFunctionWrap(x, optParam, modelParam, liftParam), ...
            innerloopParam.x0, innerloopParam.A, innerloopParam.b, ...
            innerloopParam.Aeq, innerloopParam.beq,...
            innerloopParam.lb, innerloopParam.ub, ...
            @(x)constraintFunctionWrap(x, optParam, modelParam, liftParam), ...
            innerloopParam.op ...
        );
    
% Get the joint angles
[q_star,~,~] = unpackSplines(x_star, modelParam, itpParam, outerloopParam.Time);

% Return the rmse between trajectory and human
f = outerloopParam.rmse(outerloopParam.q, q_star);

end

