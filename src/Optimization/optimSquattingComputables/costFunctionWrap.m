function [J,dJ] = costFunctionWrap(x, optParam)
%COSTFUNCTIONWRAP wraps around the CASADI generated cost function
%costFunctionSet.mexw64 and returns a dense output weighed by the cost
%function parametrization within the optParam structure.

% Call function
[J, dJ] = costFunctionSet(x);

% Densify returns
J = full(J);
dJ = full(dJ);

% Weight output
J = sum(optParam.CostFunctionWeights .* J);
dJ = sum(optParam.CostFunctionWeights .* dJ, 2);
end

