clear all;
close all; clc;

%% Import libraries and data

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
addpath('inverseOptimSquattingComputables\');

% Load segmented trajectories in Trial struct
load ../../data/3DOF/Segmentation/SegmentedTrials.mat

%% Define some simulation parameters

% Should we generate all the simulation functions and gradients or can they
% be loaded
simParam.GenerateCostAndConstraints = true;

% Give a suffix for the saved data
simParam.NumConstraintPoints=50;
% simParam.SaveSuffix = 'EqualWeights_50_ConstraintPoints';

% Define which trial we should take
simParam.TrialNumber = 1;

%% Define Interpolation Parameters, Additional Model Parameters, Additional Optimization Parameters

parameter_def_itp_model_opt_inverse;

%% Cost Function Parametrization:

% Parametrization of the compound cost function
optParam.CostFunctionWeights = [0 0 1];

% Normalisation values (load values)
load ../../data/3DOF/Optimization-Human/CostFunctionNormalisation.mat
optParam.CostFunctionNormalisation = CostFunctionNormalisation;
clear CostFunctionNormalisation

%% Generating functions for inverse optimization

% Generate or load linear constraint matrices
[A, b, Aeq, beq] = optimGenerateLinearConstraintMatricesSquatting3DOF(itpParam, optParam, modelParam);


% Code generation is time consuming, do it only if flag is set
if simParam.GenerateCostAndConstraints
    % Turn off warnings for code generation
    % MINGW Version not supported
    warning('off','all');

    % Generate nonlinear constraint computables
    inverseOptimGenerateComputableConstraintFunctionsSquatting3DOF(itpParam, optParam, modelParam);

    % Generate cost function computable
    inverseOptimGenerateComputableCostFunctionsSquatting3DOF(itpParam, optParam, modelParam);

    % Turn on warnings
    warning('on', 'all');
end