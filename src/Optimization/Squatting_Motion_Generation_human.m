clear all; close all; clc;

%% Import stuff

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

% Load model parameters
load ../../data/3DOF/Optimization-Human/squat_param.mat

% Load segmented kinematic data
load ../../data/3DOF/Segmentation/SegmentedTrials.mat

%% Define some simulation parameters

% Define which trial to take
simParam.TrialNumber = 1;

%% Define time related parameters
% Number of points
N = size(Trial(simParam.TrialNumber).q, 2);

% Sampling rate
Ts = 0.01;

% Start time
t0 = 0;

% Final time
tf = (N-1)*Ts;

% Time vector
t = linspace(t0, tf, N);


%% Animation
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
% Human starting and ending position of squat defined
q0 = Trial(simParam.TrialNumber).q(:, 1);
qc = Trial(simParam.TrialNumber).q(:, floor(Trial(simParam.TrialNumber).CrunchTimePercentage * length(Trial(simParam.TrialNumber).q)));
qf = Trial(simParam.TrialNumber).q(:, end);

T0 = FKM_3DOF_Tensor(q0, L);
Tf = FKM_3DOF_Tensor(qf, L);

% Make linear interpolation
SamplesToBottom = round(N * Trial(simParam.TrialNumber).CrunchTimePercentage);
SamplesFromBottom = N - SamplesToBottom;
q1 = [linspace(q0(1), qc(1), SamplesToBottom), linspace(qc(1), qf(1), SamplesFromBottom)];
q2 = [linspace(q0(2), qc(2), SamplesToBottom), linspace(qc(2), qf(2), SamplesFromBottom)];
q3 = [linspace(q0(3), qc(3), SamplesToBottom), linspace(qc(3), qf(3), SamplesFromBottom)];

q = [q1;q2;q3];

Animate_3DOF(q, L, Ts, opts);