function allSamplesGenerateComputableCost(timeParam,optParam,modelParam)
%ALLSAMPLESGENERATECOMPUTABLECOST generates a computable cost function set 
%for the squatting optimization with a 3DOF model, using all samples at 
%disposal.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*timeParam.NumSamples);

%% Vector of cost functions and their gradients
% Compute cost functions, and get back a vector
J = allSamplesCostFunction(x_cas, timeParam, modelParam);

% Normalise cost functions
J = J ./ optParam.CostFunctionNormalisation;

% Get cost functions' gradients
jacJ = jacobian(J, x_cas)';

% Create a casadi computable function
costFunctionSet = casadi.Function('CostFunctionSet', {x_cas}, {J, jacJ}, {'Control Points'}, {'J', 'dJ'});

%% Code generation
% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('allSamplesApproachComputables\');

% Generate a c file for the set of cost function
costFunctionSet.generate('costFunctionSet.c', casadiopts);

% Generate a mex file for the set of cost functions
mex costFunctionSet.c -DCASADI_MEX_ALWAYS_DENSE

% % Save in mat file
% save('costFunctionSet.mat', 'costFunctionSet');

% Return to previous directory
cd('..');
end

