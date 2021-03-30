%Animates a multiple sit-to-stand trajectories with data from subject
%Charlotte.

clear all; close all; clc;

%% Include libs
% Add charlotte's data
addpath("../../../data/3DOF/Charlotte")
% Add parent path
addpath("..")
% Load the parameters
load Charlotte.mat

% Take a piece of the trajectories
TimeSpan = 4078:4300;
qh = q(:, TimeSpan);
dqh = dq(:, TimeSpan);
ddqh= ddq(:, TimeSpan);
dddqh = dddq(:, TimeSpan);
% qh = q;
% dqh = dq;
% ddqh= ddq;
% dddqh = dddq;

clear q dq ddq dddq
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

%% Prepare data for animation

% Define segment lengths
L = [param.L1, param.L2, param.L3];

% Define segment relative masses
M = [param.M1, param.M2, param.M3] / param.Mtot;

% Define individial segment COM position vectors and place them in a matrix
C1 = [param.MX1; param.MY1; 0] / param.M1;
C2 = [param.MX2; param.MY2; 0] / param.M2;
C3 = [param.MX3; param.MY3; 0] / param.M3;
CMP = [C1, C2, C3];

% Find FKM
T = FKM_3DOF_Tensor(qh, L);
X = squeeze(T(1, 4, end, :));
Y = squeeze(T(2, 4, end, :));

% Find COM
COM = COM_3DOF_Tensor(qh, L, M, CMP);

%% Animate
% Create opts structure for animation
opts = struct();

% Create background plotting function for plotting knot points
opts.bgrPlot = @()backgroundPlot(X, Y, COM);

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Save movie
opts.saveas.name = "mov2";

% Call the animate function
Animate_nDOF(qh, L, Ts, opts);

%% Plotting function
function backgroundPlot(X, Y, COM)
    plot(X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end