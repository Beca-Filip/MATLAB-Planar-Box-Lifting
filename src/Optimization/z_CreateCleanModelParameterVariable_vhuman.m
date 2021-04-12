clear all; close all; clc;

% Add the data to path
addpath("../../data/3DOF/Charlotte/");

% Load human model parameters
load Charlotte.mat
load param_variable.mat

% Invert toe and heel X and Z axis, since the coordinate system of the
% kinematic data used for squat has different axes
IM = roty(180);
Markers.RTOE = (IM * Markers.RTOE.').';
Markers.RHEE = (IM * Markers.RHEE.').';

% Create new param variable
modelParam.MX1 = param.MX1;
modelParam.MX2 = param.MX2;
modelParam.MX3 = param.MX3;

modelParam.MY1 = param.MY1;
modelParam.MY2 = param.MY2;
modelParam.MY3 = param.MY3;

modelParam.M1 = param.M1;
modelParam.M2 = param.M2;
modelParam.M3 = param.M3;
modelParam.Mtot = modelParam.M1 + modelParam.M2 + modelParam.M3;

modelParam.L1 = param.L1;
modelParam.L2 = param.L2;
modelParam.L3 = param.L3;

modelParam.ZZ1 = param.ZZ1;
modelParam.ZZ2 = param.ZZ2;
modelParam.ZZ3 = param.ZZ3;

modelParam.FV1 = param.FV1;
modelParam.FV2 = param.FV2;
modelParam.FV3 = param.FV3;

modelParam.ToePosition=mean(Markers.RTOE);
modelParam.HeelPosition=mean(Markers.RHEE);


modelParam.Gravity=-9.81;
modelParam.TorqueLimits=param.taumax;
modelParam.JointLimits=[...
    [pi/4; 2*pi/3],...
    [-0.0873; 3*pi/4],... Allow negative 5 degrees
    [-pi/3; 0.0873] ... Allow positive 5 degrees
    ];

% Save new param variable
save("../../data/3DOF/Optimization-Human/squat_param.mat", "modelParam");

%%
Mtoe = mean(Markers.RTOE);
stoe = std(Markers.RTOE);
Mhee = mean(Markers.RHEE);
shee = std(Markers.RHEE);

Xtoe = Mtoe + stoe .* randn(1000, 3);
Xhee = Mhee + shee .* randn(1000, 3);

figure;
subplot(3, 1, 1)
hold on;
histogram(Xtoe(:, 1)*1000);
histogram(Xhee(:, 3)*1000);
xlabel('X Toe Position [mm]');
subplot(3, 1, 2)
hold on;
histogram(Xtoe(:, 2)*1000);
histogram(Xhee(:, 3)*1000);
xlabel('Y Toe Position [mm]');
subplot(3, 1, 3)
hold on;
histogram(Xtoe(:, 3)*1000);
histogram(Xhee(:, 3)*1000);
xlabel('Z Toe Position [mm]');