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

% Weight output
J = sum(optParam.CostFunctionWeights .* [J, J2]);
dJ = sum(optParam.CostFunctionWeights .* [dJ, dJ2], 2);
end

