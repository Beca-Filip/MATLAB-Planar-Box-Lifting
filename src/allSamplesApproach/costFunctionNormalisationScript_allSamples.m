%COSTFUNCTIONNORMALISATIONSCRIPT_ALLSAMPLES is designed to find individual
%cost function values for human data to be able to perform normalization.
clear all;
clc;

% Add model functions
addpath("../Model"); 

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

% Load modelParam variable
load('../../data/3DOF/Optimization-Human/squat_param.mat');

% Add generated functions to path
addpath('allSamplesApproachComputables\');

% Load segmented trajectories in Trial struct
load('../../data/3DOF/Segmentation/SegmentedTrials.mat');

%% Define some simulation parameters

% Should we generate all the simulation functions and gradients or can they
% be loaded
simParam.GenerateCostAndConstraints = false;

% Give a suffix for the saved data
% simParam.SaveSuffix = 'Feasible_50_ConstraintPoints';

% Define which trial we should take
simParam.TrialNumber = 1;

%% Define Interpolation Parameters, Additional Model Parameters, Additional Optimization Parameters

parameter_def_all_samples_model_opt;

%% Extract current trial's variables

% Get the current trial's trajectory 
q = Trial(simParam.TrialNumber).q;

% Get the vector
xq = reshape(q', 1, numel(q));

%% Optimization pipeline

% Generate or load linear constraint matrices
[A, b, Aeq, beq] = allSamplesGenerateLinearConstraintMatrices(timeParam, optParam, modelParam);
             
% Lower and upper boundss
One = ones(1, timeParam.NumSamples);
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
    allSamplesGenerateComputableConstraints(timeParam, optParam, modelParam);

    % Generate cost function computable
    allSamplesGenerateComputableCost(timeParam, optParam, modelParam);

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
    f_curr = costFunctionWrap(xq, optParam);

    % Memorize the values
    CostFunctionHumanValues(ii) = f_curr;
end

% Change directories and save the
CostFunctionNormalisation = CostFunctionHumanValues;
save('../../data/3DOF/allSamples/CostFunctionNormalisation.mat', 'CostFunctionNormalisation');

%% Verification

% Velocity
dq = diff(q, 1, 2) ./ timeParam.SamplingPeriod;

% Acceleration
ddq = diff(dq, 1, 2) ./ timeParam.SamplingPeriod;

% Get reduced
q = q(:, 1:end-2);
dq = dq(:, 1:end-1);

% Number of samples entering calculations
N = size(q, 2);

% Zero ext wrenches
ZEW = zeroExternalWrenches3DOF(N);

% Get GAMMA
[GAMMA, ~] = Dyn_3DOF(q, dq, ddq, ZEW, modelParam);

% Get functions
torqueF = sum(sum(GAMMA.^2, 2) ./ (modelParam.TorqueLimits').^2) / N / 3;
accelF = sum(ddq.^2, 'all') ./ N / 3;
powerF = sum(sum((dq .* GAMMA).^2, 2) ./ (modelParam.TorqueLimits').^2) / N / 3;

fprintf(['The cost function values calculated by the functions are [%.4f, %.4f, %.4f]\n'], CostFunctionHumanValues);
fprintf(['The cost function values calculated independently are    [%.4f, %.4f, %.4f]\n'], torqueF, accelF, powerF);
