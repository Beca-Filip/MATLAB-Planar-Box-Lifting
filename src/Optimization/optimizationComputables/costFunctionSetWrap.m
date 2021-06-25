function [J,dJ] = costFunctionSetWrap(x, optParam)
%COSTFUNCTIONSETWRAP wraps around the CASADI generated cost function
%costFunctionSet.mexw64 and returns a dense output out the vector function.

% Call function
[J, dJ] = costFunctionSet(x);
[J2, dJ2] = costFunctionSet2(x);

% Densify returns
J = full(J);
dJ = full(dJ);
J2 = full(J2);
dJ2 = full(dJ2);

% Weight output
J = [J, J2];
dJ = [dJ, dJ2];
end

