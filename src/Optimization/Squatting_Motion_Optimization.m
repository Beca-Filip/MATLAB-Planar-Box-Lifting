% clear all;
close all; clc;

%% Import libraries and data

% Add the data to path
addpath("../../data/3DOF/Squat");

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");  

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
load squat_param.mat

% Add generated functions to path
addpath('optimSquattingComputables\');
%% Define time related parameters
% Number of points
N = 1001;

% Start time
t0 = 0;

% Final time
tf = 1.00*3;

% Time vector
t = linspace(t0, tf, N);

% Sampling rate
Ts = (tf - t0) / (N-1);

%% Spline interpolation parameters

% Create a structure containing interpolation parameters
% Number of knots (Must be an odd number because of the way we form the initial solution)
itpParam.NumControlPoints = 15;

% Spline Knot Indices
itpParam.KnotIndices = floor(linspace(1, N, itpParam.NumControlPoints));

% Spline Knot Values - abscissa - 1st joint
itpParam.KnotValues = t(itpParam.KnotIndices);

% Order of interpolation
itpParam.InterpolationOrder = 5;

% Velocity and acceleration boundary conditions
itpParam.BoundaryConditions = [
              1, t(1), 0;       % Zero velocity at first time
              2, t(1), 0;       % Zero acceleration at first time
              1, t(end), 0;     % Zero velocity at the last time
              2, t(end), 0;     % Zero acceleration at the last time
];

% How many points to interpolate with in optimization constraints
itpParam.ItpResolutionConstraints = 50;

% How many points to interpolate with in optimization cost calculation
itpParam.ItpResolutionCost = 50;

%% Define other input parameters to the model

% Initial squatting position (upright)
modelParam.InitialAngles = [pi/2; 0; 0];

% Final squatting position (upright)
modelParam.FinalAngles = [pi/2; 0; 0];

% Crunch time in percentage of total time
modelParam.CrunchTimePercentage = 0.5;

% Final squatting neck position percentage
modelParam.CrunchNeckHeightPercentage = 0.6;

% Final squatting hip position percentage
modelParam.CrunchHipHeightPercentage = 0.4;

% Imaginary starting and ending position of squat defined
q0 = [pi/2; 0; 0];
qf = [3*pi/4; -3*pi/4; 2*pi/3];
%% Constraint tolerances

% Get default tolerance
DefaultConstraintTolerance = optimoptions('fmincon').ConstraintTolerance;

% Initial condition tolerances
TolInitialConditions = 1e-3;
optParam.MulInitialConditions = DefaultConstraintTolerance / TolInitialConditions;

% Final condition tolerances
TolFinalConditions = 1e-3;
optParam.MulFinalConditions = DefaultConstraintTolerance / TolFinalConditions;

% Crunch condition tolerances
TolCrunchConditions = 1e-3;
optParam.MulCrunchConditions = DefaultConstraintTolerance / TolCrunchConditions;

% Center of Pressure within bounds constraints
TolCOPConditions = 1e-3;  % in meters
optParam.MulCOPConditions = DefaultConstraintTolerance / TolCOPConditions;

% Torque limits constriants
TolTorqueLimits = 1e-3;
optParam.MulTorqueLimits = DefaultConstraintTolerance / TolTorqueLimits;

%% Cost Function Parametrization:

% Parametrization of the compound cost function
% If variable outsideWeights is defined, use those weights
if exist("outsideWeights")
    optParam.CostFunctionWeights = outsideWeights;
else
%     optParam.CostFunctionWeights = [0.25 0.25 0.25 0.25];
    optParam.CostFunctionWeights = [0 0 1];
%     optParam.CostFunctionWeights = zeros(1, 3);
%     optParam.CostFunctionWeights = [.25 .4 .35];
end

% Get the normalization ( Minima and Maxima, to be able to have CF's of the
% same order of magnitude ) See: Xiang, 2010
% These are set to constants but a script will be written to determine them
% automatically
load("Feasibility_Initial_15.mat");   % Loads feasible x0
load("premilinary.mat");        % Loads minima and maxima
optParam.CostFunctionMinima = CostFunctionMinima([1 2 4]);
% optParam.CostFunctionMaxima = optimCostFunctionSquatting3DOF(x0, itpParam, optParam, modelParam);
optParam.CostFunctionMaxima = CostFunctionMaxima([1 2 4]);
%% Optimization pipeline

% Generate linear constraint matrices
[A, b, Aeq, beq] = optimGenerateLinearConstraintMatricesSquatting3DOF(itpParam, optParam, modelParam);

% Turn off warnings for code generation
% MINGW Version not supported
warning('off','all');

% Generate nonlinear constraint computables
optimGenerateComputableConstraintFunctionsSquatting3DOF(itpParam, optParam, modelParam);

% Generate cost function computable
optimGenerateComputableCostFunctionSquatting3DOF(itpParam, optParam, modelParam);

% Turn on warnings
warning('on', 'all');

% Optimization options
% With gradient check
% op = optimoptions('fmincon',...   
%                   'Algorithm', 'sqp',...
%                   'Display', 'Iter', ...
%                   'MaxIter', 1e4, ...
%                   'MaxFunctionEvaluations', 2e5, ...
%                   'SpecifyObjectiveGradient', true, ...
%                   'SpecifyConstraintGradient', true,...
%                   'TolFun', 1e-3, ...
%                   'CheckGradients', true, ...
%                   'FiniteDifferenceType', 'Central', ...
%                   'FiniteDifferenceStepSize', 8e-5, ...
%                   'UseParallel', 'Always' ...
%                   );
% Without gradientcheck
op = optimoptions('fmincon',... 
                  'Algorithm', 'sqp',...
                  'Display', 'Iter', ...
                  'MaxIter', 1e4, ...
                  'MaxFunctionEvaluations', 2e5, ...
                  'SpecifyObjectiveGradient', true, ...
                  'SpecifyConstraintGradient', true,...
                  'TolFun', 1e-3, ...
                  'UseParallel', 'Always' ...
                  );

% Initial solution ( Linearly spaced knots to bottom position followed by 
% linearly spaced knots to standing position )
% q1_knot_0 = [linspace(q0(1), qf(1), (itpParam.NumControlPoints + 1) / 2),...
%              linspace(qf(1), q0(1), (itpParam.NumControlPoints + 1) / 2)];
% q1_knot_0((itpParam.NumControlPoints + 1) / 2) = [];    % Remove doubled knot
% q2_knot_0 = [linspace(q0(2), qf(2), (itpParam.NumControlPoints + 1) / 2),...
%              linspace(qf(2), q0(2), (itpParam.NumControlPoints + 1) / 2)];
% q2_knot_0((itpParam.NumControlPoints + 1) / 2) = [];    % Remove doubled knot
% q3_knot_0 = [linspace(q0(3), qf(3), (itpParam.NumControlPoints + 1) / 2),...
%              linspace(qf(3), q0(3), (itpParam.NumControlPoints + 1) / 2)];
% q3_knot_0((itpParam.NumControlPoints + 1) / 2) = [];    % Remove doubled knot
% x0 = [q1_knot_0, q2_knot_0, q3_knot_0];
%
% This solution is infeasible because of COP constraints. Getting a
% feasible solution is as easy as starting with that one and minimizing a
% constant cost function (e.g. setting all cost function weights to 0).
% That has been done and this solution will be loaded.
% load("Feasible_Initial.mat");   % Loads feasible x0
load("Feasibility_Initial_15.mat");   % Loads feasible x0

% Evaluate initial solution
[J0, ~] = costFun(x0);
[C0, Ceq0, ~, ~] = nonlinearConstr(x0);

% Print evaluations
fprintf("The inital function value is: %.4f .\n", J0);
if all(C0 < op.ConstraintTolerance) && all(abs(Ceq0) < op.ConstraintTolerance)
    fprintf("The initial solution is feasible.\n");
else
    fprintf("The initial solution is infeasible.\n");
end


% Lower and upper boundss
One = ones(1, itpParam.NumControlPoints);
lb = [modelParam.JointLimits(1, 1)*One, modelParam.JointLimits(1, 2)*One, modelParam.JointLimits(1, 3)*One];
ub = [modelParam.JointLimits(2, 1)*One, modelParam.JointLimits(2, 2)*One, modelParam.JointLimits(2, 3)*One];

% Optimization
[x_star, f_star, ef_star, out_star, lbd_star, grad_star, hess_star] = ...
        fmincon(...
            @(x)fullify(@(z)costFun(z), x), ...
            x0, A, b, Aeq, beq, lb, ub, ...
            @(x)fullify(@(z)nonlinearConstr(z), x), ...
            op ...
        );
    
%% Interpret results

% Define useful constant names
n = itpParam.NumControlPoints;
p = itpParam.InterpolationOrder;
bndcnd = itpParam.BoundaryConditions;

% Decompose optimal solution
q1_knot_star = x_star(1:n)';
q2_knot_star = x_star(n+1:2*n)';
q3_knot_star = x_star(2*n+1:3*n)';

% Get spline interpolation coefs for each joint trajectory
polycoefs1_star = splineInterpolation(itpParam.KnotValues, q1_knot_star, p, bndcnd);
polycoefs2_star = splineInterpolation(itpParam.KnotValues, q2_knot_star, p, bndcnd);
polycoefs3_star = splineInterpolation(itpParam.KnotValues, q3_knot_star, p, bndcnd);

% Get spline trajectories from coeffs
q1_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs1_star, t, 3);
q2_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs2_star, t, 3);
q3_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs3_star, t, 3);

% Pack trajectories into a single matrix
dddq_star = [q1_star(4, :); q2_star(4, :); q3_star(4, :)];
ddq_star = [q1_star(3, :); q2_star(3, :); q3_star(3, :)];
dq_star = [q1_star(2, :); q2_star(2, :); q3_star(2, :)];
q_star = [q1_star(1, :); q2_star(1, :); q3_star(1, :)];

% Pack knots into a single matrix
q_knot_star = [q1_knot_star; q2_knot_star; q3_knot_star];

%% Motion animation
% Create opts structure for animation
opts = struct();

% Define segment lengths
L = [modelParam.L1, modelParam.L2, modelParam.L3];

% Define segment relative masses
M = [modelParam.M1, modelParam.M2, modelParam.M3] / modelParam.Mtot;

% Define individial segment COM position vectors and place them in a matrix
C1 = [modelParam.MX1; modelParam.MY1; 0] / modelParam.M1;
C2 = [modelParam.MX2; modelParam.MY2; 0] / modelParam.M2;
C3 = [modelParam.MX3; modelParam.MY3; 0] / modelParam.M3;
CMP = [C1, C2, C3];

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

%% Artificial squatting motion

Animate_3DOF(q_star, L, Ts, opts);

%% Save the optimization data
save('Data_Optimization.mat', 'x_star', 'f_star', 'itpParam', 'optParam', 'modelParam');