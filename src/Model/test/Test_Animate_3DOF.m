%TEST_ANIMATE_3DOF tests the animation function of a 3DOF robot, the
%Animate_3DOF.
clear all;
close all;
clc;

%% Test default version

% Select sampling rate Ts > 0.01
Ts = 0.01;

% Num Samples
N = 1000;

% Form time vector
t = 0 : Ts : (N-1) * Ts;

% Define robot segment length
L = [0.5 0.35 0.2];

% Form joint angle row vectors and stack them vertically
q = [sin(t); cos(t); cos(2*t)];

% Call the animation function
Animate_3DOF(q, L, Ts);

%% Test with hand tool

% Create options with hand tool
opts.tool = struct("type", "hand", "length", min(L) * 0.25);

% Call the animation function
Animate_3DOF(q, L, Ts, opts);

%% Test with circle tool

% Create options with circle tool
opts.tool = struct("type", "circle", "diameter", min(L) * 0.5);

% Call the animation function
Animate_3DOF(q, L, Ts, opts);

%% Test with background function

% Create options with background plotting function as an anonymous function
% or as a .m function
opts.bgrPlot = @() plot(0.1*randn(1, 20), 0.1*randn(1, 20), 'rx', 'DisplayName', 'RandomPoints');

% Call the animation function
Animate_3DOF(q, L, Ts, opts);

%% Test generate legend function

% Create options with generate legend set to true
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animation function
Animate_3DOF(q, L, Ts, opts);