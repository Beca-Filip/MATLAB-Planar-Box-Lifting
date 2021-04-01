function optimGenerateComputableCostFunctionsSquatting3DOF(itpParam,optParam,modelParam)
%OPTIMGENERATECOMPUTABLECOSTFUNCTIONSSQUATTING3DOF generates a computable 
%compound cost function for the squatting optimization consisting of a
%linear combination of multiple cost functions.

%. It also 

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

% Compute cost functions, and get back a vector
J = optimCostFunctionSquatting3DOF(x_cas, itpParam, optParam, modelParam);

% Compute linear combination of cost functions
% J = sum(J .* optParam.CostFunctionWeights);

% Compute gradient
jacJ = jacobian(J, x_cas);

% Create casadi computable function
costFun = casadi.Function('costFun', {x_cas}, {J, jacJ}, {'Control Points'}, {'J', 'dJ'});

% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('optimSquattingComputables\');

% Generate a c file
costFun.generate('costFun.c', casadiopts);

% Generate a mex file
mex costFun.c -DCASASI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

