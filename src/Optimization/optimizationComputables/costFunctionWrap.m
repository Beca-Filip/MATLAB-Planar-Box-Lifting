function [J,dJ] = costFunctionWrap(x, optParam, modelParam, liftParam)
%COSTFUNCTIONWRAP wraps around the CASADI generated cost function
%costFunctionSet.mexw64 and returns a dense output weighed by the cost
%function parametrization within the optParam structure.

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
Inputs{end+1} = liftParam.BoxToWristVectorDuringLift;
Inputs{end+1} = liftParam.PercentageLiftOff;
Inputs{end+1} = liftParam.PercentageDropOff;
Inputs{end+1} = liftParam.WristPositionLiftOff;
Inputs{end+1} = liftParam.WristPositionDropOff;
Inputs{end+1} = liftParam.InitialAngles;
Inputs{end+1} = liftParam.FinalAngles;
Inputs{end+1} = liftParam.HeelPosition;
Inputs{end+1} = liftParam.ToePosition;
Inputs{end+1} = liftParam.BoxRectangleInitial;

% Call function
[J, dJ] = costFunctionSet(Inputs{:});
[J2, dJ2] = costFunctionSet2(Inputs{:});

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

