function optimGenerateComputableConstraintFunctionsSquatting3DOF(itpParam,optTolerance,modelParam)
%OPTIMGENERATECOMPUTABLECONSTRAINTFUNCTIONSSQUATTING3DOF generates
%computable constraints for the squatting optimization.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

% Compute constraints
[C, Ceq] = optimConstraintFunctionSquatting3DOF(x_cas, itpParam, optTolerance, modelParam);

% Compute gradients
jacC = jacobian(C, x_cas)';
jacCeq = jacobian(Ceq, x_cas)';

% Create casadi computable function
nonlinearConstr = casadi.Function('nonlinearConstr', {x_cas}, {C, Ceq, jacC, jacCeq}, {'Control Points'}, {'C', 'Ceq', 'dC', 'dCeq'});

% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('optimSquattingComputables\');

% Generate a c file
nonlinearConstr.generate('nonlinearConstr.c', casadiopts);

% Generate a mex file
mex nonlinearConstr.c -DCASASI_MEX_ALWAYS_DENSE

% Return to previous directory
cd('..');
end

