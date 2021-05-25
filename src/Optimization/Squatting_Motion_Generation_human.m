clear all; close all; clc;

%% Import stuff

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");

% Load data
load ../../data/Input-Data/Subject1_Filip_Segmented.mat


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


%% Animation

% Define segment lengths
L = [modelParam.L1,modelParam.L2,modelParam.L3,modelParam.L4,modelParam.L5,modelParam.L6];

% Define segment relative masses
M = [modelParam.M1,modelParam.M2,modelParam.M3,modelParam.M4,modelParam.M5,modelParam.M6] / modelParam.Mtot;

% Define individial segment COM position vectors and place them in a matrix
CMP = [modelParam.COM1,modelParam.COM2,modelParam.COM3,modelParam.COM4,modelParam.COM5,modelParam.COM6];

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

%% Artificial squatting motion 
% Human starting and ending position of squat defined
q0 = q(:, 1);
qlo = q(:, round(LiftParam.PercentageLiftOff * size(q, 2)));
qdo = q(:, round(LiftParam.PercentageDropOff * size(q, 2)));
qf = q(:, end);

T0 = FKM_3DOF_Tensor(q0, L);
Tf = FKM_3DOF_Tensor(qf, L);

% Make linear interpolation
TrajSamp_1 = round(LiftParam.PercentageLiftOff * size(q, 2));
TrajSamp_2 = round((LiftParam.PercentageDropOff - LiftParam.PercentageLiftOff) * size(q, 2));
TrajSamp_3 = size(q, 2) - round(LiftParam.PercentageDropOff * size(q, 2));
for ii = 1 : size(q, 1)
    qi1(ii, :) = linspace(q0(ii), qlo(ii), TrajSamp_1);
    qi2(ii, :) = linspace(qlo(ii), qdo(ii), TrajSamp_2);
    qi3(ii, :) = linspace(qdo(ii), qf(ii), TrajSamp_3);
end

qi = [qi1,qi2,qi3];

Animate_nDOF(qi, L, Ts);