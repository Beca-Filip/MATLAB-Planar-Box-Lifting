function [J] = optimCostFunctionSquatting3DOF(x,itpParam,optParam,modelParam)
%OPTIMCOSTFUNCTIONSQUATTING3DOF implements the desired cost function of a 
%3DOF model of a human performing a squat.

% Extract useful constants
n = itpParam.NumControlPoints;
t = itpParam.KnotValues;
p = itpParam.InterpolationOrder;
bndcnd = itpParam.BoundaryConditions;

% Unpack optimization variable
q1_knot = x(1:n);
q2_knot = x(n+1:2*n);
q3_knot = x(2*n+1:3*n);

% Initialize cost function vector
J = [];

%% Intermediate Computations: Spline Interpolation

% Get spline interpolation coefficients
c1 = splineInterpolation2(t, q1_knot, p, bndcnd);
c2 = splineInterpolation2(t, q2_knot, p, bndcnd);
c3 = splineInterpolation2(t, q3_knot, p, bndcnd);

% Determine the values at which to get the function value depending on the
% desired resolution
Time = linspace(t(1), t(end), itpParam.ItpResolutionCost);

% Get the function values, as well as 1st, 2nd and 3rd derivative ( by
% specifying a 3 as the function's last argument )
qdqddq1 = splineCoefToTrajectory(t, c1, Time, 3);
qdqddq2 = splineCoefToTrajectory(t, c2, Time, 3);
qdqddq3 = splineCoefToTrajectory(t, c3, Time, 3);

% Stack joint trajectories, velocities and accelerations in their own data
% structure
q = [qdqddq1(1, :); qdqddq2(1, :); qdqddq3(1, :)];
dq = [qdqddq1(2, :); qdqddq2(2, :); qdqddq3(2, :)];
ddq = [qdqddq1(3, :); qdqddq2(3, :); qdqddq3(3, :)];
dddq = [qdqddq1(4, :); qdqddq2(4, :); qdqddq3(4, :)];

%% Intermediate Computations: Joint Torques

% Generate zero external wrenches structure as required by Dyn_3DOF
ZEW = zeroExternalWrenches3DOF(itpParam.ItpResolutionCost);

% Get wrenches from Dyn_3DOF
[GAMMA, ~] = Dyn_3DOF(q, dq, ddq, ZEW, modelParam);

% Get Joint Torque cost function
JT = sum(sum(GAMMA.^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
J = [J JT];

%% Intermediate Computations: Joint Acceleration

% Get Joint Acceleration cost function
JA = sum(sum(ddq.^2)) / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
J = [J JA];

%% Intermediate Computations: Joint Jerk

% Get Joint Jerk cost function
JJ = sum(sum(dddq.^2)) / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
% J = [J JJ];

%% Intermediate Computations: Joint Power

% Get Joint Power cost function
JP = sum(sum((dq .* GAMMA).^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
J = [J JP];

%% Final Computation:

% Cost Function
% J = J;
end

