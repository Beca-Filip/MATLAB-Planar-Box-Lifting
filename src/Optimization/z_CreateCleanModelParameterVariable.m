clear all; close all; clc;

% Add the data to path
addpath("../../data");

% Load human model parameters
load param_variable.mat

% Create new param variable
modelParams.MX1 = param.MX1;
modelParams.MX2 = param.MX2;
modelParams.MX3 = param.MX3;

modelParams.MY1 = param.MY1;
modelParams.MY2 = param.MY2;
modelParams.MY3 = param.MY3;

modelParams.M1 = param.M1;
modelParams.M2 = param.M2;
modelParams.M3 = param.M3;

modelParams.L1 = param.L1;
modelParams.L2 = param.L2;
modelParams.L3 = param.L3;

modelParams.ZZ1 = param.ZZ1;
modelParams.ZZ2 = param.ZZ2;
modelParams.ZZ3 = param.ZZ3;

modelParams.FV1 = param.FV1;
modelParams.FV2 = param.FV2;
modelParams.FV3 = param.FV3;

% Save both function and new param variable
zeroEW = @(N) zeroExternalWrenches3DOF(N);

save("../../data/squat_param.mat", "param", "zeroEW");


% Create function that returns zero external wrenches variable of size N
function ZEW = zeroExternalWrenches3DOF(N)
    ZEW.FX = zeros(3, N);
    ZEW.FY = zeros(3, N);
    ZEW.FZ = zeros(3, N);
    ZEW.CX = zeros(3, N);
    ZEW.CY = zeros(3, N);
    ZEW.CZ = zeros(3, N);
end