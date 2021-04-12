function optimGenerateComputableCostFunctionsSquatting3DOF_v2(itpParam,optParam,modelParam)
%OPTIMGENERATECOMPUTABLECOSTFUNCTIONSSQUATTING3DOF_V2 generates a computable 
%cost function set for the squatting optimization with a 3DOF model.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

%% Vector of cost functions and their gradients
% Compute cost functions, and get back a vector
J = optimCostFunctionSquatting3DOF(x_cas, itpParam, modelParam);

% Normalise cost functions
J = J ./ optParam.CostFunctionNormalisation;

% Get cost functions' gradients
jacJ = jacobian(J, x_cas)';

% Create a casadi computable function
costFunctionSet = casadi.Function('costFunctionSet', {x_cas}, {J, jacJ}, {'Control Points'}, {'J', 'dJ'});

%% Code generation
% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('optimSquattingComputables\');

% Generate a c file for the set of cost function
costFunctionSet.generate('costFunctionSet.c', casadiopts);

% Generate a mex file for the set of cost functions
mex costFunctionSet.c -DCASADI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

