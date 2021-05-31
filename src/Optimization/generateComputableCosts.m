function generateComputableCosts(itpParam,optParam,modelParam,liftParam)
%GENERATECOMPUTABLECOSTS generates a computable cost function set for the 
%box-lifting optimization with a 6DOF model.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, modelParam.NJoints*itpParam.NumControlPoints);

%% Vector of cost functions and their gradients
% Compute cost functions, and get back a vector
J = costFunctions(x_cas, itpParam, modelParam,liftParam);

% Normalise cost functions
J = J ./ optParam.CostFunctionNormalisation;

% Get cost functions' gradients
jacJ = jacobian(J, x_cas)';

% Create a casadi computable function
costFunctionSet = casadi.Function('costFunctionSet', {x_cas}, {J, jacJ}, {'Control Points'}, {'J', 'dJ'});

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

% Generate a mex file for the set of cost functions
mex costFunctionSet.c -DCASADI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

