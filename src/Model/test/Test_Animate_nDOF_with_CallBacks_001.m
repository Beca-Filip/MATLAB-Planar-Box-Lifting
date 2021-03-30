%Animates a multiple sit-to-stand trajectories with data from subject
%Charlotte. Implements a callback function in animation options.

clear all; close all; clc;

%% Include libs
% Add charlotte's data
addpath("../../../data/3DOF/Charlotte")
% Add parent path
addpath("..")
% Load the parameters
load Charlotte.mat

% Take whole trajectories
qh = q;
dqh = dq;
ddqh= ddq;
dddqh = dddq;

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

%% Animate
% Create opts structure for animation
opts = struct();

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

% Create background plotting function for plotting knot points
opts.bgrPlot = @()backgroundPlot(X, Y, COM);

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a callback function
opts.callback = @(ii, handles) callbackFcn(ii, handles, COM);

% Create initialisation for handles
opts.handleInits = {@()plot(0, 0, 'x', 'Color', [.5, 0, .5], 'MarkerSize', 10, 'DisplayName', 'currentCOM');
                    @()plot(0,0, 'rv', 'DisplayName','randompt');};

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_nDOF(qh, L, Ts, opts);
%% Additional plots
FrameAxesXx = squeeze(T(1, 1, :, :));
FrameAxesXy = squeeze(T(2, 1, :, :));
FrameAxesYx = squeeze(T(1, 2, :, :));
FrameAxesYy = squeeze(T(2, 2, :, :));

Xx = [squeeze(T(1, 4, :, end))'; squeeze(T(1, 4, :, end))' + FrameAxesXx(:, end)'*0.2];
Xy = [squeeze(T(2, 4, :, end))'; squeeze(T(2, 4, :, end))' + FrameAxesXy(:, end)'*0.2];

Yx = [squeeze(T(1, 4, :, end))'; squeeze(T(1, 4, :, end))' + FrameAxesYx(:, end)'*0.2];
Yy = [squeeze(T(2, 4, :, end))'; squeeze(T(2, 4, :, end))' + FrameAxesYy(:, end)'*0.2];

hx = plot(Xx, Xy, 'r', 'LineWidth', 2);
hy = plot(Yx, Yy, 'g', 'LineWidth', 2);
%% Plotting function
function backgroundPlot(X, Y, COM)
    plot(X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end

function callbackFcn(ii, handles, COM)
    % Update COM Position
    handles(1).XData = COM(1, ii);      % X position of COM at time ii
    handles(1).YData = COM(2, ii);      % Y position of COM at time ii
    
    % Update Random Point position
    handles(2).XData = 1;
    handles(2).YData = sin(2*pi*(ii/670));      % Y position of point at time ii
end