function [C,Ceq,dC,dCeq] = constraintFunctionWrap(x, optParam)
%CONSTRAINTFUNCTIONWRAP wraps around the CASADI generated cost function
%nonlinearConstr.mexw64 and returns dense outputs.

% Call function
[C, Ceq, dC, dCeq] = nonlinearConstr(x);

% Densify returns
C = full(C);
dC = full(dC);
Ceq = full(Ceq);
dCeq = full(dCeq);

% Normalize with multipliers
C = C .* optParam.InequalityMultipliers';
dC = dC .* optParam.InequalityMultipliers;
Ceq = Ceq .* optParam.EqualityMultipliers';
dCeq = dCeq .* optParam.EqualityMultipliers;
end