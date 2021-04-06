function optimGenerateComputableCostFunctionsSquatting3DOF(itpParam,optParam,modelParam)
%OPTIMGENERATECOMPUTABLECOSTFUNCTIONSSQUATTING3DOF generates a computable 
%compound cost function for the squatting optimization consisting of a
%linear combination of multiple cost functions.

%. It also 

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

%% Vector of cost functions and their gradients
% Compute cost functions, and get back a vector
J = optimCostFunctionSquatting3DOF(x_cas, itpParam, optParam, modelParam);

% Normalise cost functions
J = (J - optParam.CostFunctionMinima) ./ (optParam.CostFunctionMaxima - optParam.CostFunctionMinima);

% Get cost functions' gradients
jacJ = jacobian(J, x_cas)';

% Create a casadi computable function
costFunctionSet = casadi.Function('costFunctionSet', {x_cas}, {J, jacJ}, {'Control Points'}, {'J', 'dJ'});

%% Compound cost function, as a weighed linear combination
% Compute linear combination of cost functions
compoundCostFunction = sum(J .* optParam.CostFunctionWeights);

% Compute gradient
jacCompoundCostFunction = jacobian(compoundCostFunction, x_cas)';

% Create casadi computable function
costFun = casadi.Function('costFun', {x_cas}, {compoundCostFunction, jacCompoundCostFunction}, {'Control Points'}, {'J', 'dJ'});

%% Code generation for both setups
% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('optimSquattingComputables\');

% Generate a c file for the set of cost function
costFunctionSet.generate('costFunctionSet.c', casadiopts);

% Generate a mex file for the set of cost functions
mex costFunctionSet.c -DCASADI_MEX_ALWAYS_DENSE

% Generate a c file for the compound cost function
costFun.generate('costFun.c', casadiopts);

% Generate a mex file for the compound cost function
mex costFun.c -DCASASI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

