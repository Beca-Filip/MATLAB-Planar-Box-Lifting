clear all; close all; clc;
%% polyCoefToTrajectory demo

% Polynomial coefficients in increasing order
c = [0, 0, 0, 0, 1/4];

% Times at which to calculate polynomials
t = -10:0.1:10;

% Variable number of outputs can be requested
[p, dp, ddp, dddp, ddddp] = polyCoefToTrajectory(c, t);

figure; hold on;
plot(t, p, 'b');
plot(t, dp, 'r');
plot(t, ddp, 'g');
plot(t, dddp, 'y');
plot(t, ddddp, 'c');
grid;

%% polyCoefToTrajectory4 demo

% Polynomial coefficients in increasing order
c = [0, 0, 0, 0, 1/4];

% Times at which to calculate polynomials
t = -10:0.1:10;

% Fixed number of outputs, however derivatives is a matrix
[p, derivatives] = polyCoefToTrajectory4(c, t, 4);

figure; hold on;
plot(t, p, 'b');
plot(t, derivatives(1, :), 'r');
plot(t, derivatives(2, :), 'g');
plot(t, derivatives(3, :), 'y');
plot(t, derivatives(4, :), 'c');
grid;


%% splineInterpolation and splineCoefToTrajectory demo

% Constants
N = 101; % Number of points
s = 5;  % How much points we skip in between knots
t0 = pi/2;     % Beginning 
tf = (2*5 + 1)*pi/2;    % Final time

% Vector of time where trajectory should be calculated
t = linspace(t0, tf, N);

% Knot points - abscissa
x = t(1:s:N);

% Knot points - ordinate
y = sin(x);

% Order of interpolation
p = 5;

% Boundary conditions - this variable has very weird formatting, I'll have
% to revisit it.
% Boundary conditions are additional conditions that have to be placed on
% the spline interpolant function or its derivatives. In this stupid
% implementation, the boundary conditions are packaged inside a numerical
% matrix that has to be of size (Number of conditions x 3). Note that
% number of conditions must be equal to the order of interpolation - 1.
%
% First column - represents the order of the derivative upon which the
% condition is placed. Usually we'll put boundary conditions on velocity
% and acceleration, so the values will be either 1 or 2.
%
% Second column - numeric value of the time upon which the condition is
% placed. Usually we will place conditions at the beginning and at the end
% of the trajectory, so the values will be t(1) or t(end).
%
% Third column - numeric value of the function's derivative. Usually we'll
% be equating things to 0.
boundaries = [
              1, t(1), 0;       % Zero velocity at first time
              2, t(1), 0;       % Zero acceleration at first time
              1, t(end), 0;     % Zero velocity at the last time
              2, t(end), 0;     % Zero acceleration at the last time
];

% Spline interpolation function returns a vector of size:
% ((Order + 1) * Number of knots)x1
c = splineInterpolation2(x, y, p, boundaries);

% Spline coef to trajectory returns the function if order is zero
f = splineCoefToTrajectory(x, c, t, 0);

figure; hold on;
plot(t, sin(t), 'b');
plot(t, f(1, :), 'r');
plot(t, f(2, :), 'y');
plot(x, y, 'og');
legend('Original', 'Interpolated', 'Location', 'Best')
grid;
title("Interpolation of sin(x)");