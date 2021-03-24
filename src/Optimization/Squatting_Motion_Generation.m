clear all; close all; clc;

%% Import stuff

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
% Starting and ending position of squat defined
q0 = [pi/2; 0; 0];
% q0 = modelParam.JointLimits(1, :)';
qf = [2*pi/3; -2*pi/3; 2*pi/3];
% qf = modelParam.JointLimits(2, :)';

T0 = FKM_3DOF_Tensor(q0, L);
Tf = FKM_3DOF_Tensor(qf, L);

% Make linear interpolation
q1 = [linspace(q0(1), qf(1), N), linspace(qf(1), q0(1), N)];
q2 = [linspace(q0(2), qf(2), N), linspace(qf(2), q0(2), N)];
q3 = [linspace(q0(3), qf(3), N), linspace(qf(3), q0(3), N)];

q = [q1;q2;q3];

Animate_3DOF(q, L, Ts, opts);