function [A, b, Aeq, beq] = allSamplesGenerateLinearConstraintMatrices(timeParam, optParam, modelParam)
%ALLSAMPLESGENERATELINEARCONSTRAINTMATRICES generates the matrices of
%linear equalities and inequalities required by the squat optimization,
%using all samples.

%% Preprocessing

% Exctract useful constants
N = timeParam.NumSamples;

% Create optimization variable that has 3*NumControlPoints elements
x_cas = casadi.SX.sym('x_cas', 1, 3*N);

% Divvy up the variables between the 3 joints
q1_cas = x_cas(1 : N);
q2_cas = x_cas(N + 1 : 2*N);
q3_cas = x_cas(2*N + 1 : 3*N);

%% Equality constraints

% Write the equality constraint functions h(x), from (h(x) = 0)
% fmincon requires Aeq*x=beq for linear constraints, meaning that 
% h(x) = Aeq*x - beq

eqCon = [
        optParam.MulInitialConditions * (q1_cas(1) - modelParam.InitialAngles(1));
        optParam.MulInitialConditions * (q2_cas(1) - modelParam.InitialAngles(2));
        optParam.MulInitialConditions * (q3_cas(1) - modelParam.InitialAngles(3));
        optParam.MulFinalConditions * (q1_cas(end) - modelParam.FinalAngles(1));
        optParam.MulFinalConditions * (q2_cas(end) - modelParam.FinalAngles(2));
        optParam.MulFinalConditions * (q3_cas(end) - modelParam.FinalAngles(3));
        ];

% Get the jacobian
jacEqCon = jacobian(eqCon, x_cas);

% Make each one computable
evalEqCon = casadi.Function('evalEqCon', {x_cas}, {eqCon});
evalJacEqCon = casadi.Function('evalJacEqCon', {x_cas}, {jacEqCon});

% If h(x) = Aeq*x - beq then Aeq = (grad_x h(x))' and beq = -h(0)
beq = -full(evalEqCon(zeros(1, length(x_cas))));
Aeq = full(evalJacEqCon(randn(1, length(x_cas))));

%% Inequality constraints

% Write the equality constraint functions g(x), from (g(x) <= 0)
% fmincon requires A*x<=b for linear constraints, meaning that g(x) = A*x - b
ineqCon = [];

% Get the jacobian
jacIneqCon = jacobian(ineqCon, x_cas);

% Make each one computable
evalIneqCon = casadi.Function('evalIneqCon', {x_cas}, {ineqCon});
evalJacIneqCon = casadi.Function('evalJacIneqCon', {x_cas}, {jacIneqCon});

% If g(x) = A*x - b then A = (grad_x g(x))' and b = -g(0)
b = -full(evalIneqCon(zeros(1, length(x_cas))));
A = full(evalJacIneqCon(randn(1, length(x_cas))));

end

