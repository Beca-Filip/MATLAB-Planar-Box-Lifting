%Animates a single sit-to-stand trajectory with data from subject
%Charlotte.

clear all; close all; clc;

%% Include libs
% Add spline library
addpath("../libs/splinePack"); 
% Add charlotte's data
addpath("../data/3DOF/Charlotte")
% Load the parameters
load Charlotte.mat

% Extract a piece of the trajectories
TimeSpan=4078:4300;
q=q(:,TimeSpan); qh = q;
dq=dq(:, TimeSpan); dqh = dq;
ddq=ddq(:, TimeSpan); ddqh = ddq;
dddq=dddq(:, TimeSpan); dddqh = dddq;
clear q dq ddq dddq
%% Define time related parameters
% Number of points
M = 223;

% Start time
t0 = 0;

% Final time
tf = 2.22*2;

% Time vector
t = linspace(t0, tf, M);

% Sampling rate
Ts = (tf - t0) / (M-1);

%% Define spline related parameters

% Number of knots
N = 11;

% Spline Knot Indices
ski = floor(linspace(1, M, N));

% Spline Knot Points - abscissa - 1st joint
skp = t(ski);

% Knot points - ordinate - 1st joint - Human Ankle Joint Coordinates
q1_knot = qh(1, ski);

% Knot points - ordinate - 2nd joint - Human Knee Joint Coordinates
q2_knot = qh(2, ski);

% Knot points - ordinate - 3rd joint - Human Hip Joint Coordinates
q3_knot = qh(3, ski);

% Order of interpolation
p = 5;

% Velocity and acceleration boundary conditions
boundaries = [
              1, t(1), 0;       % Zero velocity at first time
              2, t(1), 0;       % Zero acceleration at first time
              1, t(end), 0;     % Zero velocity at the last time
              2, t(end), 0;     % Zero acceleration at the last time
];

% Interpolate - 1st joint
c1 = splineInterpolation(skp, q1_knot, p, boundaries);

% Interpolate - 2nd joint
c2 = splineInterpolation(skp, q2_knot, p, boundaries);

% Interpolate - 3rd joint
c3 = splineInterpolation(skp, q3_knot, p, boundaries);

% Coefficients to trajectory - 1st joint
q1 = splineCoefToTrajectory(skp, c1, t, 0);

% Coefficients to trajectory - 2nd joint
q2 = splineCoefToTrajectory(skp, c2, t, 0);

% Coefficients to trajectory - 3rd joint
q3 = splineCoefToTrajectory(skp, c3, t, 0);

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

% Form knot values
q_knot = [q1_knot; q2_knot; q3_knot];

% Perform FKM for the knots
T_knot = FKM_3DOF_Tensor(q_knot, L);
X_knot = squeeze(T_knot(1, 4, end, :));
Y_knot = squeeze(T_knot(2, 4, end, :));

% Create kinematics
q = [q1; q2; q3];

% Perform FKM for all samples
T = FKM_3DOF_Tensor(q, L);
X = squeeze(T(1, 4, end, :));
Y = squeeze(T(2, 4, end, :));

% Perform COM FKM for the knots
COM_knot = COM_3DOF_Tensor(q_knot, L, M, CMP);

% Perform COM FKM for the whole trajectory
COM = COM_3DOF_Tensor(q, L, M, CMP);

% Create background plotting function for plotting knot points
% opts.bgrPlot = @()plot(X_knot, Y_knot, 'go', X, Y, 'DisplayName', 'KnotPoints&Traj', 'LineWidth', 1.3);
opts.bgrPlot = @()backgroundPlot(X_knot, Y_knot, X, Y, COM_knot, COM);

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_3DOF(q, L, Ts, opts);

%% Plotting function
function backgroundPlot(X_knot, Y_knot, X, Y, COM_knot, COM)
    plot(X_knot, Y_knot, 'go', X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM_knot(1, :), COM_knot(2, :), 'bo', COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end