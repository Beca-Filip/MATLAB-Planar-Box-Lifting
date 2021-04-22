function [C,Ceq,dC,dCeq] = constraintFunctionWrap(x)
%CONSTRAINTFUNCTIONWRAP wraps around the CASADI generated cost function
%nonlinearConstr.mexw64 and returns dense outputs.

% Call function
[C, Ceq, dC, dCeq] = nonlinearConstr(x);

% Densify returns
C = full(C);
dC = full(dC);
Ceq = full(Ceq);
dCeq = full(dCeq);

end

