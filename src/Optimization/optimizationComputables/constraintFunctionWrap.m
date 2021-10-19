function [C,Ceq,dC,dCeq] = constraintFunctionWrap(x, optParam, modelParam, liftParam)
%CONSTRAINTFUNCTIONWRAP wraps around the CASADI generated cost function
%nonlinearConstr.mexw64 and returns dense outputs.

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
[C, Ceq, dC, dCeq] = nonlinearConstr(Inputs{:});

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