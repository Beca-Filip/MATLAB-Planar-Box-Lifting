%Animates a multiple sit-to-stand trajectories with data from subject
%Charlotte. Implements a callback function in animation options with
%quivers.

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
opts.callback = @(ii, handles) callbackFcn(ii, handles, COM, T);

% Create initialisation for handles
opts.handleInits = {@()plot(0, 0, 'x', 'Color', [.5, 0, .5], 'MarkerSize', 10, 'DisplayName', 'currentCOM');
                    @()plot(0,0, 'rv', 'DisplayName','randompt');
                    @()quiver(0, 0, 0, 0, 'r', 'DisplayName', 'X-axes', 'AutoScaleFactor', 0.3);
                    @()quiver(0, 0, 0, 0, 'g', 'DisplayName', 'Y-axes', 'AutoScaleFactor', 0.3);};

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_nDOF(qh, L, Ts, opts);

%% Plotting function
function backgroundPlot(X, Y, COM)
    plot(X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end

function callbackFcn(ii, handles, COM, T)
    % Update COM Position
    handles(1).XData = COM(1, ii);      % X position of COM at time ii
    handles(1).YData = COM(2, ii);      % Y position of COM at time ii
    
    % Update Random Point position
    handles(2).XData = 1;
    handles(2).YData = sin(2*pi*(ii/670));      % Y position of point at time ii
    
    % For quivers be selective about replotting to not slow down
    thresh = 1e-2;
    Ti = T(:, :, :, ii);
    if ii > 2
        T_prev = T(:, :, :, ii-1);
    else
        T_prev = Ti + thresh + 1;
    end
    
    % Replot only if
    if sum(abs(reshape( Ti - T_prev, 1, []))) > thresh
    % Update reference frames position and x-axes
    handles(3).XData = T(1, 4, :, ii);  % X position of all coordinate frames at time ii
    handles(3).YData = T(2, 4, :, ii);  % Y position of all coordinate frames at time ii
    handles(3).UData = T(1, 1, :, ii);  % X component of X-axes unit vectors of all frames at time ii
    handles(3).VData = T(2, 1, :, ii);  % Y component of X-axes unit vectors of all frames at time ii
    
    % Update reference frames position and y-axes
    handles(4).XData = T(1, 4, :, ii);  % X position of all coordinate frames at time ii
    handles(4).YData = T(2, 4, :, ii);  % Y position of all coordinate frames at time ii
    handles(4).UData = T(1, 2, :, ii);  % X component of Y-axes unit vectors of all frames at time ii
    handles(4).VData = T(2, 2, :, ii);  % Y component of Y-axes unit vectors of all frames at time ii
    end
end