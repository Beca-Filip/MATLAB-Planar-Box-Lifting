function [J,dJ] = costFunWrap(x)
%COSTFUNWRAP wraps around the CASADI generated cost function costFun.mexw64
%and returns dense outputs.

% Call function
[J, dJ] = costFun(x);

% Densify returns
J = full(J);
dJ = full(dJ);

end

