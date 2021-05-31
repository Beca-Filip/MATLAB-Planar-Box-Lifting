function [A, b, Aeq, beq] = generateLinearConstraints(itpParam, optParam, modelParam, liftParam)
%GENERATELINEARCONSTRAINTS generates the matrices of linear equalities and 
%inequalities required by the motion optimization.

%% Preprocessing

% Extract useful constants
Ncp = itpParam.NumControlPoints;
Tknots = itpParam.KnotValues;
ItpOrder = itpParam.InterpolationOrder;
BndCnd = itpParam.BoundaryConditions;
NJ = modelParam.NJoints;

% Create optimization variable that has 3*NumControlPoints elements
x_cas = casadi.SX.sym('x_cas', 1, NJ*itpParam.NumControlPoints);

% Segment length vector
L = [];
for ii = 1 : NJ
    L = [L modelParam.(['L', num2str(ii)])];
end

% Extract knots of individual joints
q_knots = cell(1, NJ);
for ii = 1 : NJ
    q_knots{ii} = x_cas(1 + (ii-1) * Ncp : ii * Ncp);
end

%% Equality constraints

% Write the equality constraint functions h(x), from (h(x) = 0)
% fmincon requires Aeq*x=beq for linear constraints, meaning that 
% h(x) = Aeq*x - beq

% Initialize empty vector
eqCon = [];

% Stack vertically initial conditions
for ii = 1 : NJ
    eqCon = [eqCon;
        optParam.MulInitialConditions * (q_knots{ii}(1) - liftParam.InitialAngles(ii))
    ];
end

% Stack vertically final conditions
for ii = 1 : NJ
    eqCon = [eqCon;
        optParam.MulFinalConditions * (q_knots{ii}(end) - liftParam.FinalAngles(ii))
    ];
end

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

