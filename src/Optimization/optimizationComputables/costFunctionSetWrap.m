function [J,dJ] = costFunctionSetWrap(x, optParam, modelParam)
%COSTFUNCTIONSETWRAP wraps around the CASADI generated cost function
%costFunctionSet.mexw64 and returns a dense output out the vector function.

% Prepare function inputs
Inputs = {x};
for ii = 1 : 6
    Inputs{end+1} = modelParam.(['L', num2str(ii)]);
    Inputs{end+1} = modelParam.(['M', num2str(ii)]);
    Inputs{end+1} = modelParam.(['COM', num2str(ii)]);
    Inputs{end+1} = modelParam.(['ZZ', num2str(ii)]);
end
Inputs{end+1} = modelParam.JointLimits;
Inputs{end+1} = modelParam.TorqueLimits;

% Call function
[J, dJ] = costFunctionSet(Inputs{:});
[J2, dJ2] = costFunctionSet2(Inputs{:});

% Densify returns
J = full(J);
dJ = full(dJ);
J2 = full(J2);
dJ2 = full(dJ2);

% Normalize output
J = [J, J2] ./ optParam.CostFunctionNormalisation;
dJ = [dJ, dJ2] ./ optParam.CostFunctionNormalisation;
end

