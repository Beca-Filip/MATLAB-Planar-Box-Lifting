%COSTFUNCTIONNORMALIZATIONSCRIPT_HUMAN is designed to find individual cost
%function extrema to be able to perform normalization.
clear all;

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
load('../../data/3DOF/Optimization-Human/squat_param.mat')

% Add generated functions to path
addpath('optimSquattingComputables\');

% Load segmented trajectories in Trial struct
load ../../data/3DOF/Segmentation/SegmentedTrials.mat

%% Define some simulation parameters

% Should we generate all the simulation functions and gradients or can they
% be loaded
simParam.GenerateCostAndConstraints = false;

% Give a suffix for the saved data
% simParam.SaveSuffix = 'Feasible_50_ConstraintPoints';

% Define which trial we should take
simParam.TrialNumber = 1;

%% Define Interpolation Parameters, Additional Model Parameters, Additional Optimization Parameters

parameter_def_itp_model_opt;

%% Extract current trial's variables

% Get the current trial's trajectory 
q = Trial(simParam.TrialNumber).q;

% Extract the variables
% Take NumControlPoints equally spaced points of the trajectories 
indices = round(linspace(1, size(q, 2), itpParam.NumControlPoints));
% Get the q's corresponding to those indices
q_knots = q(:, indices)';
% Get the q's in row vector form, joint by joint
q_knots = q_knots(:).';

%% Optimization pipeline

% Generate or load linear constraint matrices
[A, b, Aeq, beq] = optimGenerateLinearConstraintMatricesSquatting3DOF(itpParam, optParam, modelParam);
             
% Lower and upper boundss
One = ones(1, itpParam.NumControlPoints);
lb = [modelParam.JointLimits(1, 1)*One, modelParam.JointLimits(1, 2)*One, modelParam.JointLimits(1, 3)*One];
ub = [modelParam.JointLimits(2, 1)*One, modelParam.JointLimits(2, 2)*One, modelParam.JointLimits(2, 3)*One];

% Define normalisation to be only ones
optParam.CostFunctionNormalisation = [1 1 1];

% Code generation is time consuming, do it only if flag is set
if simParam.GenerateCostAndConstraints
    % Turn off warnings for code generation
    % MINGW Version not supported
    warning('off','all');

    % Generate nonlinear constraint computables
    optimGenerateComputableConstraintFunctionsSquatting3DOF_v2(itpParam, optParam, modelParam);

    % Generate cost function computable
    optimGenerateComputableCostFunctionsSquatting3DOF_v2(itpParam, optParam, modelParam);

    % Turn on warnings
    warning('on', 'all');
end

% Look at the values for each function separately and memorize cost
% function values
CostFunctionHumanValues = zeros(1, length(optParam.CostFunctionNormalisation));
optParam.CostFunctionWeights = zeros(1, length(optParam.CostFunctionNormalisation));

% For each cost function
for ii = 1 :  length(optParam.CostFunctionNormalisation)
    
    % Set the current cost functions weight to 1 and others to 0
    optParam.CostFunctionWeights = zeros(1, length(optParam.CostFunctionNormalisation));
    optParam.CostFunctionWeights(ii) = 1;
        
    % Current cost function value
    f_curr = costFunctionWrap(q_knots, optParam);

    % Memorize the values
    CostFunctionHumanValues(ii) = f_curr;
end

% Change directories and save the
CostFunctionNormalisation = CostFunctionHumanValues;
save('../../data/3DOF/Optimization-Human/CostFunctionNormalisation.mat', 'CostFunctionNormalisation');

