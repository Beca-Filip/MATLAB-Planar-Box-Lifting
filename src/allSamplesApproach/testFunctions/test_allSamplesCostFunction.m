clear all; close all; clc;

%%

% Import model parameters in the modelParam structure
load('../../../data/3DOF/Squat/squat_param.mat');

% Import CASADI
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

% Import kinematics and dynamics functions
addpath('../../Model/')

% Import the directory with the functions to test
addpath('../')

%% Test

% Number of samples of trajectory (pick random num)
timeParam.NumSamples = 287;

% Sampling period
timeParam.SamplingPeriod = 0.01;

% Create a symbolic variable
x = casadi.SX.sym('x', 1, 3*287);

% Push it through function
J = allSamplesCostFunction(x, timeParam, modelParam);