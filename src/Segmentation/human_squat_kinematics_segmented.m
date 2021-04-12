%HUMAN_SQUAT_KINEMATICS_SEGMENTED loads the 7DOF kinematic data of the
%human squatting model. It performs a transformation defined by an
%anonymous function dof7to3, to transform the 7DOF model to an equivalent
%3DOF model in the plane, with joints in the ankle, knee and hip.
%Model parameters are loaded from Charlotte (segment lengths, inertias and
%masses).
%The movement collection of the squats, which is segmented within script
%Segment_Data and where sample-indices of the segmentation are memorized
%within SquatTimeSegments.mat, is then animated segment by segment instead
%of the whole collection at a time.
%The individual movements are packed in a structure called Trial, which
%contains the joint angles for the whole trajectory, the ratio of minimum
%to maximum end-effector height, and the ratio of crunch time to total
%time.
clear all; close all; clc;

%% Import libraries and data

% Add the data to path
addpath("../../data/3DOF/Squat");
% Load model data
load squat_param.mat
load Kinematics_meas_Sb4_Tr2.mat

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");


% Modify to fit 3DOF
dof7to3 = @(vq) [sum(vq(1:2, :));vq(3, :);vq(4, :)];
dof7to3 = @(vq) [vq(2, :);vq(3, :);sum(vq(4:5, :), 1)];
figure;
for ii = 1 : size(q, 1)
    subplot(size(q, 1), 1, ii)
    plot(q(ii, :));
    %title('theta ' + num2str(ii))
end
q = dof7to3(q);
dq = dof7to3(dq);
ddq = dof7to3(ddq);

% Load segmented trajectories
load ../../data/3DOF/Segmentation/SquatTimeSegments.mat

%% Model and animation parameters loading


% Define segment lengths
L = [modelParam.L1, modelParam.L2, modelParam.L3];

% Define segment relative masses
Mass = [modelParam.M1, modelParam.M2, modelParam.M3] / modelParam.Mtot;

% Define individial segment COM position vectors and place them in a matrix
C1 = [modelParam.MX1; modelParam.MY1; 0] / modelParam.M1;
C2 = [modelParam.MX2; modelParam.MY2; 0] / modelParam.M2;
C3 = [modelParam.MX3; modelParam.MY3; 0] / modelParam.M3;
CMP = [C1, C2, C3];

% Create opts structure for animation
opts = struct();

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};
%% Animation

% For each segment
for nbseg = 1 : size(INDICES, 1)
    
    % Take only a portion of the trajectories
    qh = q(:, INDICES(nbseg, 1) : INDICES(nbseg, 2));
    dqh = dq(:, INDICES(nbseg, 1) : INDICES(nbseg, 2));
    ddqh = ddq(:, INDICES(nbseg, 1) : INDICES(nbseg, 2));
    
    %% Define time related parameters
    % Number of points
    M = size(qh, 2);

    % Start time
    t0 = 0;

    % Final time
    tf = (size(qh, 2)-1) * 0.01;

    % Time vector
    t = linspace(t0, tf, M);

    % Sampling rate
    Ts = (tf - t0) / (M-1);

    %% Animate
    % Find FKM
    T = FKM_3DOF_Tensor(qh, L);
    X_neck = squeeze(T(1, 4, end, :));
    Y_neck = squeeze(T(2, 4, end, :));
    
    X_hip = squeeze(T(1, 4, end-1, :));
    Y_hip = squeeze(T(2, 4, end-1, :));

    % Find COM
    COM = COM_3DOF_Tensor(qh, L, Mass, CMP);

    % Create background plotting function for plotting knot points
    opts.bgrPlot = @()backgroundPlot(X_neck, Y_neck, COM);

    % Call the animate function
    Animate_3DOF(qh, L, Ts, opts);
    
    % Pause before next iteration
    pause;
    
    %% Save in trials data structure
    % Save kinematics
    Trial(nbseg).q = qh;
    Trial(nbseg).dq = dqh;
    Trial(nbseg).ddq = ddqh;
    
    % Save ratios
    Trial(nbseg).CrunchNeckHeightPercentage = min(Y_neck) / max(Y_neck);
    Trial(nbseg).CrunchHipHeightPercentage = min(Y_hip) / max(Y_hip);
    Trial(nbseg).CrunchTimePercentage = ...
    (MINIMA_INDICES(nbseg) - INDICES(nbseg, 1)) ./ (INDICES(nbseg, 2) - INDICES(nbseg, 1));

    % Print ratios
    fprintf('Ratios: [%.4f, %.4f, %.4f]\n', Trial(nbseg).CrunchNeckHeightPercentage, ...
    Trial(nbseg).CrunchHipHeightPercentage, Trial(nbseg).CrunchTimePercentage);
    
end

%% Save trials
% Change directory for saves
cd('../../data/3DOF/');

% Make directory if it doesn't already exist
if ~exist('Segmentation', 'file')
    mkdir Segmentation
end

% Enter directory
cd('Segmentation');

% Save trials
save('SegmentedTrials.mat', 'Trial');

% Go back to original directory
cd(strrep(mfilename('fullpath'), mfilename, ''));

%% Plotting function
function backgroundPlot(X, Y, COM)
    plot(X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end