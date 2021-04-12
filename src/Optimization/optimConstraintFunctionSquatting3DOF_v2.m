function [C, Ceq, constraintInfo] = optimConstraintFunctionSquatting3DOF_v2(x,itpParam,modelParam)
%OPTIMCONSTRAINTFUNCTIONSQUATTING3DOF_V2 implements the constraints on a 3DOF
%model of a human performing a squat. Returns the vector of values of the
%inequality and equality constraints, as well as a structure containing the
%one word description of the constraint and the number of elements
%corresponding to that constraint.

% Extract useful constants
n = itpParam.NumControlPoints;
t = itpParam.KnotValues;
p = itpParam.InterpolationOrder;
bndcnd = itpParam.BoundaryConditions;

% Interpolation for constraints
N = itpParam.ItpResolutionConstraints;
tc = linspace(t(1), t(end), N);

% Unpack optimization variable
q1_knot = x(1:n);
q2_knot = x(n+1:2*n);
q3_knot = x(2*n+1:3*n);


%% Intermediate computations: Spline Interpolation

% Compute polynomial coefficients
polycoeffs1 = splineInterpolation2(t, q1_knot, p, bndcnd);
polycoeffs2 = splineInterpolation2(t, q2_knot, p, bndcnd);
polycoeffs3 = splineInterpolation2(t, q3_knot, p, bndcnd);

% Compute interpolation of joint angles, as well as velocities and
% accelerations (the 2 at the end)
qdqddq1 = splineCoefToTrajectory(t, polycoeffs1, tc, 2);
qdqddq2 = splineCoefToTrajectory(t, polycoeffs2, tc, 2);
qdqddq3 = splineCoefToTrajectory(t, polycoeffs3, tc, 2);

% Stack the joint angles, velocities and accelerations in their own data
% structure
q = [qdqddq1(1, :); qdqddq2(1, :); qdqddq3(1, :)];
dq = [qdqddq1(2, :); qdqddq2(2, :); qdqddq3(2, :)];
ddq = [qdqddq1(3, :); qdqddq2(3, :); qdqddq3(3, :)];

%% Intermediate Computations: Final State Neck and Hip Heights

% % Find the final joint angles
% qf = [q1_knot(end); q2_knot(end); q3_knot(end)];
% 
% % Make a segment length vector
% L = [modelParam.L1; modelParam.L2; modelParam.L3];
% 
% % Call the appropriate FKM function for the final state
% Tf = FKM_3DOF_Cell(qf, L);
% 
% % Get final neck height
% nhf = Tf{end, 1}(2, 4);
% 
% % Get final hip height
% hhf = Tf{end-1, 1}(2, 4);

% Find the time index where crunch is happening
icrunch = round(N * modelParam.CrunchTimePercentage);

% Get the joint angles at crunch time
qc = q(:, icrunch);

% Make a segment length vector
L = [modelParam.L1; modelParam.L2; modelParam.L3];

% Call the appropriate FKM function for the crunch state
Tc = FKM_3DOF_Cell(qc, L);

% Get crunch neck height
nhc = Tc{end, 1}(2, 4);

% Get crunch hip height
hhc = Tc{end-1, 1}(2, 4);


%% Intermediate Computation: Position of Center of Pressure

% Generate the zero external wrenches structure
ZEW = zeroExternalWrenches3DOF(size(q, 2));

% Compute the position of the COP 
COP = COP_3DOF_Matrix(q,dq,ddq,ZEW,modelParam);

% Take into account only the X coordinate
XCOP = COP(1, :);

% Get the thesholds
XCOP_high = max(modelParam.HeelPosition(1, 1), modelParam.ToePosition(1, 1));
XCOP_low  = min(modelParam.HeelPosition(1, 1), modelParam.ToePosition(1, 1));

%% Intermediate Computation: Joint Torques

% Compute the joint torques
[GAMMA, ~] = Dyn_3DOF(q,dq,ddq,ZEW,modelParam);

%% Initialize description structure

constraintInfo.Inequalities = struct([]);
constraintInfo.Equalities = struct([]);

%% Inequality constraints (Add constraint 1-by-1 because of implementation)

% Initialize empty vector
C = [];

% Crunch conditions
C = [C; nhc - modelParam.CrunchNeckHeightPercentage * sum(L)];
C = [C; hhc - modelParam.CrunchHipHeightPercentage * sum(L(1:end-1))];
% Add to description
constraintInfo.Inequalities(1).Description = 'CrunchConditions';
constraintInfo.Inequalities(1).Amount = length(nhc) + length(hhc);

% COP conditions
C = [C; (XCOP - XCOP_high)'];
C = [C; (-XCOP + XCOP_low)'];
% Add to description
constraintInfo.Inequalities(2).Description = 'COPConditions';
constraintInfo.Inequalities(2).Amount = 2*length(XCOP);


% Torque Limits
C = [C; (GAMMA(1, :)' - modelParam.TorqueLimits(1))];
C = [C; (-GAMMA(1, :)' - modelParam.TorqueLimits(1))];
C = [C; (GAMMA(2, :)' - modelParam.TorqueLimits(2))];
C = [C; (-GAMMA(2, :)' - modelParam.TorqueLimits(2))];
C = [C; (GAMMA(3, :)' - modelParam.TorqueLimits(3))];
C = [C; (-GAMMA(3, :)' - modelParam.TorqueLimits(3))];
% Add to description
constraintInfo.Inequalities(3).Description = 'TorqueLimits';
constraintInfo.Inequalities(3).Amount = 6*length(GAMMA(1, :));

%% Equality constraints

Ceq = [];
end

