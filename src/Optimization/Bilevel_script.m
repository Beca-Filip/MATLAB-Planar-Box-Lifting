clear all;
close all; clc;

% Commit 6/23/2021
% Commit 6/25/2021
% Commit 6/25/2021b

%% Import libraries and data

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");

% Add the generated functions
addpath("optimizationComputables")

% Add CASADI library
if ismac                            
    % Code to run on Mac platform
    addpath('../../libs/casadi-osx-matlabR2015a-v3.5.1')
elseif isunix
    % Code to run on Linux platform
    addpath('../../libs/casadi-linux-matlabR2015a-v3.5.1')
elseif ispc
    % Code to run on Windows platform
    addpath('../../libs/casadi-windows-matlabR2016a-v3.5.5')
else
    disp('Platform not supported')
end

% Load data
load ../../data/Input-Data/Subject1_Filip_Segmented.mat


%% Define time related parameters
% Number of points
N = size(q, 2);

% Sampling rate
Ts = 0.01;

% Start time
t0 = 0;

% Final time
tf = (N-1)*Ts;

% Time vector
Time = linspace(t0, tf, N);

%% Spline interpolation parameters

% Create a structure containing interpolation parameters
% Number of knots (Must be an odd number because of the way we form the initial solution)
itpParam.NumControlPoints = 10;

% Spline Knot Indices
itpParam.KnotIndices = floor(linspace(1, N, itpParam.NumControlPoints));

% Spline Knot Values - abscissa - 1st joint
itpParam.KnotValues = Time(itpParam.KnotIndices);

% Order of interpolation
itpParam.InterpolationOrder = 5;

% Velocity and acceleration boundary conditions
itpParam.BoundaryConditions = [
              1, Time(1), 0;       % Zero velocity at first time
              2, Time(1), 0;       % Zero acceleration at first time
              1, Time(end), 0;     % Zero velocity at the last time
              2, Time(end), 0;     % Zero acceleration at the last time
];

% How many points to interpolate with in optimization constraints
itpParam.ItpResolutionConstraints = 50;

% How many points to interpolate with in optimization cost calculation
itpParam.ItpResolutionCost = 50;

%% Define other input parameters to the model

% Number of joints
modelParam.NJoints = 6;

%% Constraint tolerances

% Get default tolerance
% DefaultConstraintTolerance = optimoptions('fmincon').ConstraintTolerance;
DefaultConstraintTolerance = 1e-3;
optParam.DefaultConstraintTolerance = DefaultConstraintTolerance;

% Initial condition tolerances
TolInitialConditions = 1e-3;
optParam.MulInitialConditions = DefaultConstraintTolerance / TolInitialConditions;

% Final condition tolerances
TolFinalConditions = 1e-3;
optParam.MulFinalConditions = DefaultConstraintTolerance / TolFinalConditions;

% Wrist Lift Off condition tolerances
TolWristPositionLiftOff = 2e-2;
optParam.MulWristPositionLiftOff = DefaultConstraintTolerance / TolWristPositionLiftOff;

% Wrist Drop off condition tolerances
TolWristPositionDropOff = 2e-2;
optParam.MulWristPositionDropOff = DefaultConstraintTolerance / TolWristPositionDropOff;

% Center of Pressure within bounds constraints
TolCOPConditions = 1e-3;  % in meters
optParam.MulCOPConditions = DefaultConstraintTolerance / TolCOPConditions;

% Torque limits constriants
TolTorqueLimits = 1e-3;
optParam.MulTorqueLimits = DefaultConstraintTolerance / TolTorqueLimits;

% Collision constraints
TolCollisionConstraints = 1e-2;
optParam.MulCollisionConstraints = DefaultConstraintTolerance / TolCollisionConstraints;
% optParam.MulCollisionConstraints = 0;

% Box above table at second-to-last control point constraint
TolBoxAboveAtSTL = 1e-6;
optParam.MulBoxAboveAtSTL = DefaultConstraintTolerance / TolBoxAboveAtSTL;

%% Constraint Multipliers, Cost Function Parametrization, Cost Function Normalization

% Compute constraints for random inputs just to get the constraint info
[~, ~, constraintInfo] = constraintFunctions(rand(1, modelParam.NJoints*itpParam.NumControlPoints), itpParam, modelParam, LiftParam);
% Get multipliers for constraints so as to fit the desired constraint tolerances
InequalityMultipliers = getMultipliersForTolerance(constraintInfo.Inequalities, optParam);
EqualityMultipliers = getMultipliersForTolerance(constraintInfo.Equalities, optParam);
% Store multipliers within optParam
optParam.InequalityMultipliers = InequalityMultipliers;
optParam.EqualityMultipliers = EqualityMultipliers;

load ../../data/Output-Data/Cost-Normalization/CostNormalizationHuman.mat
optParam.CostFunctionNormalisation = CostNormalization;
%% Define innerloop parameters

% Generate or load linear constraint matrices
[A, b, Aeq, beq] = generateLinearConstraints(itpParam, optParam, modelParam, LiftParam);

% Innerloop optimization options
op = optimoptions('fmincon',...   
                  'Algorithm', 'sqp',...
                  'ConstraintTolerance', optParam.DefaultConstraintTolerance, ...
                  'Display', 'none', ...
                  'MaxIter', 1e4, ...
                  'MaxFunctionEvaluations', 2e5, ...
                  'SpecifyObjectiveGradient', true, ...
                  'SpecifyConstraintGradient', true,...
                  'TolFun', 1e-3, ...
                  'UseParallel', 'Always' ...
                  );

% Load feasible initial solution
ll = q(:, itpParam.KnotIndices).';
ll = ll(:).';
x0 = ll;

% Lower and upper boundss
One = ones(1, itpParam.NumControlPoints);
lb = [];
ub = [];

for ii = 1 : modelParam.NJoints
    lb = [lb One*modelParam.JointLimits(1, ii)];
end

for ii = 1 : modelParam.NJoints
    ub = [ub One*modelParam.JointLimits(2, ii)];
end

%  Get the innerloop param structure
innerloopParam.A = A;
innerloopParam.b = b;
innerloopParam.Aeq = Aeq;
innerloopParam.beq = beq;
innerloopParam.op = op;
innerloopParam.x0 = x0;
innerloopParam.lb = lb;
innerloopParam.ub = ub;

clear A b Aeq beq op x0


%% Define outerloop parameters
% Define rmse function
outerloopParam.rmse = @(a,b) sqrt(sum((a-b).^2 / numel(a), 'all'));

% Pack time vector and joint angles vector
outerloopParam.q = q;
outerloopParam.Time = Time;

% Sum of all parameters = 1
Aeq = ones(1, 8);
beq = 1;
A = [];
b = [];

% lower bounds
lb = zeros(1, 8);
ub = [];

% Start
% x0 = ones(1, 8) / 8;
x0 = rand(1, 8); x0 = x0 / sum(x0);
% x0 = [0.0068    0.0003    0.4236    0.0028    0.1321    0.1842    0.0209    0.2293];

% Outerloop optimization options
op = optimoptions('fmincon',...   
                  'Algorithm', 'sqp',...
                  'ConstraintTolerance', 1e-3, ...
                  'Display', 'Iter', ...
                  'MaxIter', 15, ...
                  'MaxFunctionEvaluations', 2e5, ...
                  'TolFun', 1e-3, ...
                  'UseParallel', 'Always' ...
                  );

% Optimization
[x_star, f_star, ef_star, out_star, lbd_star, grad_star, hess_star] = ...
        fmincon(...
            @(c)Bilevel_outer(c, itpParam, modelParam, optParam, LiftParam, innerloopParam, outerloopParam), ...
            x0, A, b, Aeq, beq, lb, ub, ...
            [], ...
            op ...
        );

%
save('Bilevel/Try013.mat');