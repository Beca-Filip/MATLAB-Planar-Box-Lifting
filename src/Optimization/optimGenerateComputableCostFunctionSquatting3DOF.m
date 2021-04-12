function optimGenerateComputableCostFunctionsSquatting3DOF(itpParam,optParam,modelParam)
%OPTIMGENERATECOMPUTABLECOSTFUNCTIONSSQUATTING3DOF generates a computable 
%compound cost function for the squatting optimization consisting of a
%linear combination of multiple cost functions.

%. It also 

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

%% Compound cost function, as a weighed linear combination
% Compute cost functions, and get back a vector
J = optimCostFunctionSquatting3DOF(x_cas, itpParam, modelParam);

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

% Generate a c file for the compound cost function
costFun.generate('costFun.c', casadiopts);

% Generate a mex file for the compound cost function
mex costFun.c -DCASASI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

