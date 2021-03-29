clear all; close all; clc;

%% Import stuff

% Add the data to path
addpath("../../data/3DOF/Squat");
% Load model data
load squat_param.mat
load Kinematics_meas_Sb4_Tr2.mat

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");

% Rename
qh = q;
dqh = dq;
ddqh = ddq;
clear q dq ddq

%% Animate

% Parameters
L = ones(1, 7);     % Segment lengths
L(1)=1;
L(2)=1.5;
L(3)=1.5;
Ts = 0.01;          % Sampling time

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", max(L)*0.5);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_nDOF(qh, L, Ts, opts);