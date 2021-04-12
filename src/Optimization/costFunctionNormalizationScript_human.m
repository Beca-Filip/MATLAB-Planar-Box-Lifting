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
%%
