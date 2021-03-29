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
% 
% Take whole trajectories
qh = q;
dqh = dq;
ddqh= ddq;


clear q dq ddq
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

% Find FKM
T = FKM_3DOF_Tensor(qh, L);
X = squeeze(T(1, 4, end, :));
Y = squeeze(T(2, 4, end, :));

% Find COM
COM = COM_3DOF_Tensor(qh, L, M, CMP);

% Create background plotting function for plotting knot points
opts.bgrPlot = @()backgroundPlot(X, Y, COM);

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_3DOF(qh, L, Ts, opts);

%% Plotting function
function backgroundPlot(X, Y, COM)
    plot(X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end