function [C,Ceq,dC,dCeq] = nonlinearConstrWrap(x)
%NONLINEARCONSTRWRAP wraps around the CASADI generated cost function
%nonlinearConstr.mexw64 and returns dense outputs.

% Call function
[C, dC, Ceq, dCeq] = nonlinearConstr(x);

% Densify returns
C = full(C);
dC = full(dC);
Ceq = full(Ceq);
dCeq = full(dCeq);

end

