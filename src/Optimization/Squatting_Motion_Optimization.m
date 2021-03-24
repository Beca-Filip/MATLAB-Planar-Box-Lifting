clear all; close all; clc;

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
% Number of knots
itpParam.NumControlPoints = 11;

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

% Initial squatting position
modelParam.InitialAngles = [pi/2; 0; 0];

% Final squatting neck position percentage
modelParam.FinalNeckHeightPercentage = 0.6;

% Final squatting hip position percentage
modelParam.FinalHipHeightPercentage = 0.4;

% Imaginary starting and ending position of squat defined
q0 = [pi/2; 0; 0];
qf = [3*pi/4; -3*pi/4; 2*pi/3];
%% Constraint tolerances

% Get default tolerance
DefaultConstraintTolerance = optimoptions('fmincon').ConstraintTolerance;

% Initial condition tolerances
TolInitialConditions = 1e-3;
optTolerance.MulInitialConditions = DefaultConstraintTolerance / TolInitialConditions;

% Final condition tolerances
TolFinalConditions = 1e-3;
optTolerance.MulFinalConditions = DefaultConstraintTolerance / TolFinalConditions;
%% Optimization pipeline

% Generate linear constraint matrices
[A, b, Aeq, beq] = optimGenerateLinearConstraintMatricesSquatting3DOF(itpParam, optTolerance, modelParam);

% Turn off warnings for code generation
% MINGW Version not supported
warning('off','all');

% Generate nonlinear constraint computables
optimGenerateComputableConstraintFunctionsSquatting3DOF(itpParam, optTolerance, modelParam);

% Generate cost function computable
optimGenerateComputableCostFunctionSquatting3DOF(itpParam, optTolerance, modelParam);

% Turn on warnings
warning('on', 'all');

% Optimization options
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
              
% Initial solution
q1_knot_0 = linspace(q0(1), qf(1), itpParam.NumControlPoints);
q2_knot_0 = linspace(q0(2), qf(2), itpParam.NumControlPoints);
q3_knot_0 = linspace(q0(3), qf(3), itpParam.NumControlPoints);
x0 = [q1_knot_0, q2_knot_0, q3_knot_0];

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

pause;

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
q1_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs1_star, t, 2);
q2_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs2_star, t, 2);
q3_star = splineCoefToTrajectory(itpParam.KnotValues, polycoefs3_star, t, 2);

% Pack trajectories into a single matrix
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