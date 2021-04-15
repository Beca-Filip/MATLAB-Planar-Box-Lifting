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

%% Interpolate

% Get knots
q1_knot = Trial(simParam.TrialNumber).q(1, itpParam.KnotIndices);
q2_knot = Trial(simParam.TrialNumber).q(2, itpParam.KnotIndices);
q3_knot = Trial(simParam.TrialNumber).q(3, itpParam.KnotIndices);

% Get coefficients
c1 = splineInterpolation2(itpParam.KnotValues, q1_knot, itpParam.InterpolationOrder, itpParam.BoundaryConditions);
c2 = splineInterpolation2(itpParam.KnotValues, q2_knot, itpParam.InterpolationOrder, itpParam.BoundaryConditions);
c3 = splineInterpolation2(itpParam.KnotValues, q3_knot, itpParam.InterpolationOrder, itpParam.BoundaryConditions);

% Get interpolation
[qdqddq1] = splineCoefToTrajectory(itpParam.KnotValues, c1, itpParam.KnotValues, 3);
[qdqddq2] = splineCoefToTrajectory(itpParam.KnotValues, c2, itpParam.KnotValues, 3);
[qdqddq3] = splineCoefToTrajectory(itpParam.KnotValues, c3, itpParam.KnotValues, 3);

% Stack in vector
q = [qdqddq1(1, :);qdqddq2(1, :);qdqddq3(1, :)];
dq = [qdqddq1(2, :);qdqddq2(2, :);qdqddq3(2, :)];
ddq = [qdqddq1(3, :);qdqddq2(3, :);qdqddq3(3, :)];
dddq = [qdqddq1(4, :);qdqddq2(4, :);qdqddq3(4, :)];

% Compare with human
figure;

subplot(3, 1, 1)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).q(1, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, q(1, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint angle [rad]');
title(['Ankle joint interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 2)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).q(2, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, q(2, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint angle [rad]');
title(['Knee joint interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 3)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).q(3, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, q(3, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint angle [rad]');
title(['Hip joint interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

%% Velocities

figure;

subplot(3, 1, 1)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).dq(1, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, dq(1, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint velocity [rad/s]');
title(['Ankle joint velocity interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 2)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).dq(2, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, dq(2, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint velocity [rad/s]');
title(['Knee joint velocity interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 3)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).dq(3, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, dq(3, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint velocity [rad/s]');
title(['Hip joint velocity interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

%% Accelerations

figure;

subplot(3, 1, 1)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).ddq(1, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, ddq(1, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint acceleration [rad/s^2]');
title(['Ankle joint acceleration interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 2)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).ddq(2, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, ddq(2, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint acceleration [rad/s^2]');
title(['Knee joint acceleration interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);

subplot(3, 1, 3)
hold on;
plot(itpParam.KnotValues, Trial(simParam.TrialNumber).ddq(3, itpParam.KnotIndices), 'DisplayName', 'Original');
plot(itpParam.KnotValues, ddq(3, :), '--', 'DisplayName', 'Interpolation');
legend;
xlabel('time [s]');
ylabel('joint acceleration [rad/s^2]');
title(['Hip joint acceleration interpolation with ' num2str(length(itpParam.KnotIndices), "%d") ' knots.']);
