function [J,dJ] = costFunctionWrap(x, optParam)
%COSTFUNCTIONWRAP wraps around the CASADI generated cost function
%costFunctionSet.mexw64 and returns a dense output weighed by the cost
%function parametrization within the optParam structure.

% Call function
[J, dJ] = costFunctionSet(x);
[J2, dJ2] = costFunctionSet2(x);

% Densify returns
J = full(J);
dJ = full(dJ);
J2 = full(J2);
dJ2 = full(dJ2);

% Normalize Output
J = [J, J2] ./ optParam.CostFunctionNormalisation;
dJ = [dJ, dJ2] ./ optParam.CostFunctionNormalisation;

% Weight output
J = sum(optParam.CostFunctionWeights .* J);
dJ = sum(optParam.CostFunctionWeights .* dJ, 2);

end

