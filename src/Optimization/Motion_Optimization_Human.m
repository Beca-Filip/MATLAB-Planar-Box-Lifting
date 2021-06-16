clear all;
close all; clc;

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

%% Define some simulation parameters

% Should we generate all the simulation functions and gradients or can they
% be loaded
simParam.GenerateCost = false;
simParam.GenerateConstraints = false;

% Give a suffix for the saved data
% simParam.SaveSuffix = 'MinPower_50CP';
optParam.CostFunctionWeights = [0 0 1 0];

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
itpParam.NumControlPoints = 17;

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
DefaultConstraintTolerance = optimoptions('fmincon').ConstraintTolerance;

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

%% Cost Function Parametrization:

% Parametrization of the compound cost function
% optParam.CostFunctionWeights = [0 0 0 0];
% optParam.CostFunctionWeights = [0.9032    0.0814    0.0000    0.0154];

% If we're re-generating the model, please don't forget to normalize the
% cost functions
if simParam.GenerateCost || simParam.GenerateConstraints
    costFunctionNormalizationScript_human;
end

% CostNormalization = [1 1 1 1];
load ../../data/Output-Data/Cost-Normalization/CostNormalizationHuman.mat
optParam.CostFunctionNormalisation = CostNormalization;
%% Optimization pipeline

% Generate or load linear constraint matrices
[A, b, Aeq, beq] = generateLinearConstraints(itpParam, optParam, modelParam, LiftParam);


% Code generation is time consuming, do it only if flag is set
if simParam.GenerateConstraints
    % Turn off warnings for code generation
    % MINGW Version not supported
    warning('off','all');

    % Generate nonlinear constraint computables
    generateComputableConstraints(itpParam, optParam, modelParam, LiftParam);
    
    % Turn on warnings
    warning('on', 'all');
end

if simParam.GenerateCost
    % Turn off warnings for code generation
    % MINGW Version not supported
    warning('off','all');
    
    % Generate cost function computable
    generateComputableCosts(itpParam, optParam, modelParam, LiftParam);
    
    % Turn on warnings
    warning('on', 'all');
end

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
%                   'FiniteDifferenceStepSize', 1e-4, ...
%                   'UseParallel', 'Always' ...
%                   );
% % Without
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

% Load feasible initial solution
ll = q(:, itpParam.KnotIndices).';
ll = ll(:).';
x0 = ll;
% load('../../data/Output-Data/Optimization-Results/OptResults_Feasible_50CP.mat');
% x0 = OptResults.Results.x_star;
% clear OptResults

% Evaluate initial solution
[J0, dJ0] = costFunctionWrap(x0, optParam);
[C0, Ceq0, dC0, dCeq0] = nonlinearConstr(x0);
LC0 = A * x0' - b;
LCeq0 = Aeq * x0' - beq;

% Print evaluations
fprintf("The inital function value is: %.4e .\n", J0);
if ~all(C0 < op.ConstraintTolerance)    
    fprintf("The initial solution is infeasible because of NLIneq. Max. constr. val.: %.4e; Tol: %.4e\n", max(C0), op.ConstraintTolerance);
end
if ~all(abs(Ceq0) < op.ConstraintTolerance)
    fprintf("The initial solution is infeasible because of NLEq. Max. constr. val.: %.4e; Tol: %.4e\n", max(abs(Ceq0)), op.ConstraintTolerance);
end
if ~all(LC0 < op.ConstraintTolerance)
    fprintf("The initial solution is infeasible because of LIneq. Max. constr. val.: %.4e; Tol: %.4e\n", max(LC0), op.ConstraintTolerance);
end
if ~all(abs(LCeq0) < op.ConstraintTolerance)
    fprintf("The initial solution is infeasible because of LEq. Max. constr. val.: %.4e; Tol: %.4e\n", max(abs(LCeq0)), op.ConstraintTolerance);
end
if all(C0 < op.ConstraintTolerance) && all(abs(Ceq0) < op.ConstraintTolerance) && ...
   all(LC0 < op.ConstraintTolerance) && all(abs(LCeq0) < op.ConstraintTolerance)
    fprintf("The initial solution is feasible.\n");
end

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

% Optimization
[x_star, f_star, ef_star, out_star, lbd_star, grad_star, hess_star] = ...
        fmincon(...
            @(x)costFunctionWrap(x, optParam), ...
            x0, A, b, Aeq, beq, lb, ub, ...
            @(x)constraintFunctionWrap(x), ...
            op ...
        );
    
%% Interpret results

% Define useful constant names
Ncp = itpParam.NumControlPoints;
Tknots =  itpParam.KnotValues;
ItpOrder = itpParam.InterpolationOrder;
BndCnd = itpParam.BoundaryConditions;

% Number of joints ( = Num. Opt Variables / Num. Control Points)
NJ = floor(length(x_star) / Ncp);

% Extract knots of individual joints
q_knots_star = cell(1, NJ);
for ii = 1 : NJ
    q_knots_star{ii} = x_star(1 + (ii-1) * Ncp : ii * Ncp);
end

% Get spline coeffs
polycoeffs_star = cell(1, NJ);
for ii = 1 : NJ
    polycoeffs_star{ii}  = splineInterpolation2(Tknots, q_knots_star{ii}, ItpOrder, BndCnd);
end

% Get trajectories, velocities, accelerations, and jerks and separate them
q_tva = cell(1, NJ);
cell_q_star = cell(1, NJ);
cell_dq_star = cell(1, NJ);
cell_ddq_star = cell(1, NJ);
cell_dddq_star = cell(1, NJ);

for ii = 1 : NJ
    q_tva{ii} = splineCoefToTrajectory(Tknots, polycoeffs_star{ii}, Time, 3);
    cell_q_star{ii} = q_tva{ii}(1, :).'; % Transpose for later
    cell_dq_star{ii} = q_tva{ii}(2, :).'; % Transpose for later
    cell_ddq_star{ii} = q_tva{ii}(3, :).'; % Transpose for later 
    cell_dddq_star{ii} = q_tva{ii}(4, :).'; % Transpose for later    
end

% Merge trajectories velocities and accelerations into one matrix
q_star = [cell_q_star{:}].'; % Merge and transpose so rows correspond to single joint trajectories
dq_star = [cell_dq_star{:}].'; % Merge and transpose so rows correspond to single joint velocities
ddq_star = [cell_ddq_star{:}].'; % Merge and transpose so rows correspond to single joint accelerations
dddq_star = [cell_dddq_star{:}].'; % Merge and transpose so rows correspond to single joint jerks

%% Motion animation
% Create opts structure for animation
opts = struct();

% Segment length vector
L = [];
for ii = 1 : NJ
    L = [L modelParam.(['L', num2str(ii)])];
end

% Segment relative masses
M = [];
for ii = 1 : NJ
    M = [M modelParam.(['M', num2str(ii)])];
end
M = M / modelParam.Mtot;

% Define individial segment COM position vectors and place them in a matrix
CMP = [];
for ii = 1 : NJ
    CMP = [CMP modelParam.(['COM', num2str(ii)])];
end

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

%% Artificial squatting motion

% Animate_nDOF(q_star, L, Ts, opts);
Animate_Lifting(q_star,L,Ts,LiftParam);

%% Save the optimization data inside structure

% Save the parameters within structure
OptResults.ub = ub;
OptResults.lb = lb;
OptResults.itpParam = itpParam;
OptResults.modelParam = modelParam;
OptResults.optParam = optParam;
OptResults.LiftParam = LiftParam;
OptResults.Results.x_star = x_star;
OptResults.Results.f_star = f_star;
OptResults.Results.ef_star = ef_star;
OptResults.Results.lbd_star = lbd_star;

%% Use the save function
% If save is desired
if exist('simParam') && isfield(simParam, 'SaveSuffix')
    
    % Change directory to data/Output-Data
    cd('../../data/');
    if ~exist('Output-Data', 'dir')
        mkdir Output-Data
    end
    cd('Output-Data');
    
    % Change directory to data/Output-Data/Optimization-Results
    if ~exist('Optimization-Results', 'dir')
        mkdir Optimization-Results
    end
    cd('Optimization-Results')
    
    % Save with adequate suffix
    save(['OptResults_' simParam.SaveSuffix '.mat'], 'OptResults');
    

% Go back to original directory
cd(strrep(mfilename('fullpath'), mfilename, ''));
end