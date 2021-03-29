function [J] = optimCostFunctionSquatting3DOF(x,itpParam,optTolerance,modelParam)
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

%% Intermediate Computations: Spline Interpolation

% Get spline interpolation coefficients
c1 = splineInterpolation2(t, q1_knot, p, bndcnd);
c2 = splineInterpolation2(t, q2_knot, p, bndcnd);
c3 = splineInterpolation2(t, q3_knot, p, bndcnd);

% Determine the values at which to get the function value depending on the
% desired resolution
Time = linspace(t(1), t(end), itpParam.ItpResolutionCost);

% Get the function values, as well as derivative and 2nd derivative ( by
% specifying a 2 at the end of the function call )
qdqddq1 = splineCoefToTrajectory(t, c1, Time, 2);
qdqddq2 = splineCoefToTrajectory(t, c2, Time, 2);
qdqddq3 = splineCoefToTrajectory(t, c3, Time, 2);

% Stack joint trajectories, velocities and accelerations in their own data
% structure
q = [qdqddq1(1, :); qdqddq2(1, :); qdqddq3(1, :)];
dq = [qdqddq1(2, :); qdqddq2(2, :); qdqddq3(2, :)];
ddq = [qdqddq1(3, :); qdqddq2(3, :); qdqddq3(3, :)];

%% Intermediate Computations: Joint Torques

% Generate zero external wrenches structure as required by Dyn_3DOF
ZEW = zeroExternalWrenches3DOF(itpParam.ItpResolutionCost);

% Get wrenches from Dyn_3DOF
[GAMMA, ~] = Dyn_3DOF(q, dq, ddq, ZEW, modelParam);

%% Final Computation:

% Cost Function
J = -sum(sum(GAMMA.^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

end

