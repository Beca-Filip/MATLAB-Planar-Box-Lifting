function inverseOptimGenerateComputableConstraintFunctionsSquatting3DOF(itpParam,optParam,modelParam)
%INVERSEOPTIMGENERATECOMPUTABLECONSTRAINTFUNCTIONSSQUATTING3DOF generates
%computable constraints for the squatting optimization.

% Create symbolic variables
x_cas = casadi.SX.sym('x_cas', 1, 3*itpParam.NumControlPoints);

% Compute constraints
[C, Ceq, constraintInfo] = optimConstraintFunctionSquatting3DOF_v2(x_cas, itpParam, modelParam);

% Apply multipliers for constraints so as to fit the desired constraint
% tolerances
C = applyMultiplierForTolerance(C, constraintInfo.Inequalities, optParam);
Ceq = applyMultiplierForTolerance(Ceq, constraintInfo.Equalities, optParam);

% Simplify
% fprintf("Simplification...\n")
% tic
% if ~isempty(C); C = simplify(C); end
% if ~isempty(Ceq); Ceq = simplify(Ceq); end
% fprintf("Simplification time: %.4f[s].\n", toc);

% Compute gradients
jacC = jacobian(C, x_cas)';
jacCeq = jacobian(Ceq, x_cas)';

% Simplify
% fprintf("Simplification...\n")
% tic
% if ~isempty(jacC); jacC = simplify(jacC); end
% if ~isempty(jacCeq); jacCeq = simplify(jacCeq); end
% fprintf("Simplification time: %.4f[s].\n", toc);


% Create casadi computable function
nonlinearConstr = casadi.Function('nonlinearConstr', {x_cas}, {C, Ceq, jacC, jacCeq}, {'Control Points'}, {'C', 'Ceq', 'dC', 'dCeq'});

% Set code generation options
casadiopts = struct("mex", true);

% Change directory
cd('inverseOptimSquattingComputables\');

% Generate a c file
% nonlinearConstr.generate('nonlinearConstr.c', casadiopts);

% Generate a mex file
% mex nonlinearConstr.c -largeArrayDims

% Save inside a mat file
save('nonlinearConstr.mat', 'nonlinearConstr');

% Return to previous directory
cd('..');
end

function C = applyMultiplierForTolerance(C, Info, optParam)
%APPLYMULTIPLIERFORTOLERANCE multiplies constraint values in C by the
%multipliers stocked in the optParam structure, depending on their type
%given in the Info structure.

% Initialize a counter
cnt = 0;

% For each type of constraint described in info
for ii = 1 : length(Info)
    
    % Take the constraints corresponding to the described type, and
    % multiply them by the multiplier stored inside the optimization
    % parameters
    C(cnt + 1 : cnt + Info(ii).Amount) = ...
    optParam.(['Mul' Info(ii).Description]) * C(cnt + 1 : cnt + Info(ii).Amount);

    % Update the count variable
    cnt  = cnt + Info(ii).Amount;
end
end