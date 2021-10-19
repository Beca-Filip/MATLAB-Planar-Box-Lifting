function generateComputableCosts(itpParam,optParam,modelParam,liftParam)
%GENERATECOMPUTABLECOSTS generates a computable cost function set for the 
%box-lifting optimization with a 6DOF model.

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

%% Vector of cost functions and their gradients
% Compute cost functions, and get back a vector
J = costFunctions(x_cas, itpParam, modelParam,liftParam);

% Removed 7/2/2021 as normalization was transferred to costFunctionWrap
% % Normalise cost functions
% J = J ./ optParam.CostFunctionNormalisation;

% Get cost functions' gradients
jacJ = jacobian(J, x_cas)';


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

% Create a casadi computable function
numper = 4;
costFunctionSet = casadi.Function('costFunctionSet', Inputs, {J(1:numper), jacJ(:, 1:numper)}, Input_names, {'J', 'dJ'});
costFunctionSet2 = casadi.Function('costFunctionSet2', Inputs, {J(1+numper:end), jacJ(:, numper+1:end)}, Input_names, {'J', 'dJ'});

%% Code generation
% Set code generation options
casadiopts = struct("mex", true);

% Create directory if it doesn't exist
if ~exist('optimizationComputables', 'dir')
    mkdir optimizationComputables
end

% Change directory
cd('optimizationComputables\');

% Generate a c file for the set of cost function
costFunctionSet.generate('costFunctionSet.c', casadiopts);
costFunctionSet2.generate('costFunctionSet2.c', casadiopts);

% Generate a mex file for the set of cost functions
mex costFunctionSet.c -DCASADI_MEX_ALWAYS_DENSE
mex costFunctionSet2.c -DCASADI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

