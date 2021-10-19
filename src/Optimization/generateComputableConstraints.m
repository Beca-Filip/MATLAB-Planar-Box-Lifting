function generateComputableConstraints(itpParam,optParam,modelParam,liftParam)
%GENERATECOMPUTABLECONSTRAINTS generates computable constraints as well as 
%their derivatives for the desired optimization.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, modelParam.NJoints*itpParam.NumControlPoints);

% Create symbolic variables for model parameters
for ii = 1 : 6
    modelParam.(['L', num2str(ii)]) = casadi.SX.sym(['modelParam.L', num2str(ii)], 1, 1);
    modelParam.(['M', num2str(ii)]) = casadi.SX.sym(['modelParam.M', num2str(ii)], 1, 1);
    modelParam.(['COM', num2str(ii)]) = casadi.SX.sym(['modelParam.COM', num2str(ii)], 3, 1);
    modelParam.(['ZZ', num2str(ii)]) = casadi.SX.sym(['modelParam.ZZ', num2str(ii)], 1, 1);
end
modelParam.JointLimits = casadi.SX.sym('modelParam.JointLimits', 2, 6);
modelParam.TorqueLimits = casadi.SX.sym('modelParam.TorqueLimits', 1, 6);

% Create symbolic variables for task parameters
liftParam.BoxToWristVectorDuringLift = casadi.SX.sym('liftParam.BoxToWristVectorDuringLift', 3, 1);
liftParam.PercentageLiftOff = casadi.SX.sym('liftParam.PercentageLiftOff', 1, 1);
liftParam.PercentageDropOff = casadi.SX.sym('liftParam.PercentageDropOff', 1, 1);
liftParam.WristPositionLiftOff = casadi.SX.sym('liftParam.WristPositionLiftOff', 3, 1);
liftParam.WristPositionDropOff = casadi.SX.sym('liftParam.WristPositionDropOff', 3, 1);
liftParam.InitialAngles = casadi.SX.sym('liftParam.InitialAngles', 6, 1);
liftParam.FinalAngles = casadi.SX.sym('liftParam.FinalAngles', 6, 1);
liftParam.HeelPosition = casadi.SX.sym('liftParam.HeelPosition', 1, 1);
liftParam.ToePosition = casadi.SX.sym('liftParam.ToePosition', 1, 1);
liftParam.BoxRectangleInitial = casadi.SX.sym('liftParam.BoxRectangleInitial', 1, 4);

% Compute constraints
[C, Ceq, ~] = constraintFunctions(x_cas, itpParam, modelParam, liftParam);

% Compute gradients
jacC = jacobian(C, x_cas)';
jacCeq = jacobian(Ceq, x_cas)';

% Prepare function inputs
Input_names = {'Control Points'};
Inputs = {x_cas};
for ii = 1 : 6
    Input_names{end+1} = ['L', num2str(ii)];
    Inputs{end+1} =  modelParam.(['L', num2str(ii)]);
    Input_names{end+1} = ['M', num2str(ii)];
    Inputs{end+1} = modelParam.(['M', num2str(ii)]);
    Input_names{end+1} = ['COM', num2str(ii)];
    Inputs{end+1} = modelParam.(['COM', num2str(ii)]);
    Input_names{end+1} = ['ZZ', num2str(ii)];
    Inputs{end+1} = modelParam.(['ZZ', num2str(ii)]);
end
Input_names{end+1} = ['JointLimits', num2str(ii)];
Inputs{end+1} = modelParam.JointLimits;
Input_names{end+1} = ['TorqueLimits', num2str(ii)];
Inputs{end+1} = modelParam.TorqueLimits;


Input_names{end+1} = ['BoxToWristVectorDuringLift', num2str(ii)];
Inputs{end+1} = liftParam.BoxToWristVectorDuringLift;
Input_names{end+1} = ['PercentageLiftOff', num2str(ii)];
Inputs{end+1} = liftParam.PercentageLiftOff;
Input_names{end+1} = ['PercentageDropOff', num2str(ii)];
Inputs{end+1} = liftParam.PercentageDropOff;
Input_names{end+1} = ['WristPositionLiftOff', num2str(ii)];
Inputs{end+1} = liftParam.WristPositionLiftOff;
Input_names{end+1} = ['WristPositionDropOff', num2str(ii)];
Inputs{end+1} = liftParam.WristPositionDropOff;
Input_names{end+1} = ['InitialAngles', num2str(ii)];
Inputs{end+1} = liftParam.InitialAngles;
Input_names{end+1} = ['FinalAngles', num2str(ii)];
Inputs{end+1} = liftParam.FinalAngles;
Input_names{end+1} = ['HeelPosition', num2str(ii)];
Inputs{end+1} = liftParam.HeelPosition;
Input_names{end+1} = ['ToePosition', num2str(ii)];
Inputs{end+1} = liftParam.ToePosition;
Input_names{end+1} = ['BoxRectangleInitial', num2str(ii)];
Inputs{end+1} = liftParam.BoxRectangleInitial;
% Create casadi computable function
% nonlinearConstr = casadi.Function('nonlinearConstr', {x_cas}, {C, Ceq, jacC, jacCeq}, {'Control Points'}, {'C', 'Ceq', 'dC', 'dCeq'});
nonlinearConstr = casadi.Function('nonlinearConstr', Inputs, {C, Ceq, jacC, jacCeq}, Input_names, {'C', 'Ceq', 'dC', 'dCeq'});

% Set code generation options
casadiopts = struct("mex", true);

% Create directory if it doesn't exist
if ~exist('optimizationComputables', 'dir')
    mkdir optimizationComputables
end


% Change directory
mkdir optimizationComputables

cd('optimizationComputables\');

% Generate a c file
nonlinearConstr.generate('nonlinearConstr.c', casadiopts);

% Generate a mex file
mex nonlinearConstr.c -largeArrayDims

% Return to previous directory
cd('..');
end

% function C = applyMultiplierForTolerance(C, Info, optParam)
% %APPLYMULTIPLIERFORTOLERANCE multiplies constraint values in C by the
% %multipliers stocked in the optParam structure, depending on their type
% %given in the Info structure.
% 
% % Initialize a counter
% cnt = 0;
% 
% % For each type of constraint described in info
% for ii = 1 : length(Info)
%     
%     % Take the constraints corresponding to the described type, and
%     % multiply them by the multiplier stored inside the optimization
%     % parameters
%     C(cnt + 1 : cnt + Info(ii).Amount) = ...
%     optParam.(['Mul' Info(ii).Description]) * C(cnt + 1 : cnt + Info(ii).Amount);
% 
%     % Update the count variable
%     cnt  = cnt + Info(ii).Amount;
% end
% end
function mulvec = getMultipliersForTolerance(Info, optParam)
%GETMULTIPLIERSFORTOLERANCE generates a vector of multipliers of constraint
%function values in C by the multipliers stocked in the optParam structure,
%depending on their type given in the Info structure.

% Initialize a counter
cnt = 0;

% Vector of multipliers
mulvec = [];

% For each type of constraint described in info
for ii = 1 : length(Info)
    
    % Take the constraints corresponding to the described type, and
    % multiply them by the multiplier stored inside the optimization
    % parameters
    mulvec(cnt + 1 : cnt + Info(ii).Amount) = ...
    optParam.(['Mul' Info(ii).Description]) * ones(1, Info(ii).Amount);

    % Update the count variable
    cnt  = cnt + Info(ii).Amount;
end
end