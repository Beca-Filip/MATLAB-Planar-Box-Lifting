clear all; close all; clc;

%% Include spline lib
addpath("../../../libs/splinePack");


%% Define time related parameters
% Number of points
M = 501;

% Start time
t0 = 0;

% Final time
tf = 4;

% Time vector
t = linspace(t0, tf, M);

% Sampling rate
Ts = (tf - t0) / (M-1);

%% Define spline related parameters

% Number of knots
N = 11;

% Indices
si = floor(linspace(1, M, N));

% Knot points - abscissa - 1st joint
sk = t(si);

% Knot points - ordinate - 1st joint
q1_knot = cos(sk-pi/8);

% Knot points - ordinate - 2nd joint
q2_knot = sin(sk);

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
c1 = splineInterpolation(sk, q1_knot, p, boundaries);

% Interpolate - 2nd joint
c2 = splineInterpolation(sk, q2_knot, p, boundaries);

% Coefficients to trajectory - 1st joint
q1 = splineCoefToTrajectory(sk, c1, t, 0);

% Coefficients to trajectory - 2nd joint
q2 = splineCoefToTrajectory(sk, c2, t, 0);

%% Animate
% Create opts structure for animation
opts = struct();

% Define segment lengths
L = [0.5, 0.3];

% Form knot values
q_knot = [q1_knot; q2_knot];

% Perform FKM for the knots
T_knot = FKM_2DOF_Tensor(q_knot, L);
X_knot = squeeze(T_knot(1, 4, end, :));
Y_knot = squeeze(T_knot(2, 4, end, :));

% Create kinematics
q = [q1; q2];

% Perform FKM for all samples
T = FKM_2DOF_Tensor(q, L);
X = squeeze(T(1, 4, end, :));
Y = squeeze(T(2, 4, end, :));

% Create background plotting function for plotting knot points
opts.bgrPlot = @()plot(X_knot, Y_knot, 'go', X, Y, 'DisplayName', 'KnotPoints&Traj', 'LineWidth', 2);


% Create a tool option for aesthetics
opts.tool = struct("type", "hand", "length", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_2DOF(q, L, Ts, opts);