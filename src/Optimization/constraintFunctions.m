function [C, Ceq, constraintInfo] = constraintFunctions(x,itpParam,modelParam,liftParam)
%constraintFunctions implements the constraints on a 6DOF of a human 
%performing a box-lifting task. Returns the vector of values of the
%inequality and equality constraints, as well as a structure containing the
%one word description of the constraint and the number of elements
%corresponding to that constraint.

% Extract useful constants
Ncp = itpParam.NumControlPoints;
Tknots = itpParam.KnotValues;
ItpOrder = itpParam.InterpolationOrder;
BndCnd = itpParam.BoundaryConditions;

% Interpolation for constraints
Npts = itpParam.ItpResolutionConstraints;
Time = linspace(Tknots(1), Tknots(end), Npts);

% Number of joints ( = Num. Opt Variables / Num. Control Points)
NJ = floor(length(x) / Ncp);

% Segment length vector
L = [];
for ii = 1 : NJ
    L = [L modelParam.(['L', num2str(ii)])];
end

% Extract knots of individual joints
q_knots = cell(1, NJ);
for ii = 1 : NJ
    q_knots{ii} = x(1 + (ii-1) * Ncp : ii * Ncp);
end

%% Intermediate computations: Spline Interpolation

% Get spline coeffs
polycoeffs = cell(1, NJ);
for ii = 1 : NJ
    polycoeffs{ii}  = splineInterpolation2(Tknots, q_knots{ii}, ItpOrder, BndCnd);
end

% Get trajectories, velocities and accelerations and separate them
q_tva = cell(1, NJ);
cell_q = cell(1, NJ);
cell_dq = cell(1, NJ);
cell_ddq = cell(1, NJ);

for ii = 1 : NJ
    q_tva{ii} = splineCoefToTrajectory(Tknots, polycoeffs{ii}, Time, 2);
    cell_q{ii} = q_tva{ii}(1, :).'; % Transpose for later
    cell_dq{ii} = q_tva{ii}(2, :).'; % Transpose for later
    cell_ddq{ii} = q_tva{ii}(3, :).'; % Transpose for later    
end

% Merge trajectories velocities and accelerations into one matrix
q = [cell_q{:}].'; % Merge and transpose so rows correspond to single joint trajectories
dq = [cell_dq{:}].'; % Merge and transpose so rows correspond to single joint velocities
ddq = [cell_ddq{:}].'; % Merge and transpose so rows correspond to single joint accelerations

%% Intermediate Computations: Lift Off and Drop Off wrist cartesian positions

% Find the time index where lift off is happening
iLiftOff = round(Npts * liftParam.PercentageLiftOff);
% Find the time index where drop off is happening
iDropOff = round(Npts * liftParam.PercentageDropOff);

% Get the joint angles at lift off time
qlo = q(:, iLiftOff);
% Get the joint angles at drop off time
qdo = q(:, iDropOff);

% Find the Cartesian configuration of the robot ad lift-off time
Tlo = FKM_nDOF_Cell(qlo, L);
% Find the Cartesian configuration of the robot ad lift-off time
Tdo = FKM_nDOF_Cell(qdo, L);

%% Intermediate Computation: Position of Center of Pressure

% % Generate the zero external wrenches structure
% ZEW = zeroExternalWrenches3DOF(size(q, 2));
% 
% % Compute the position of the COP 
% COP = COP_3DOF_Matrix(q,dq,ddq,ZEW,modelParam);
% 
% % Take into account only the X coordinate
% XCOP = COP(1, :);
% 
% % Get the thesholds
% XCOP_high = max(modelParam.HeelPosition(1, 1), modelParam.ToePosition(1, 1));
% XCOP_low  = min(modelParam.HeelPosition(1, 1), modelParam.ToePosition(1, 1));

%% Intermediate Computation: Joint Torques

% % Compute the joint torques
% [GAMMA, ~] = Dyn_3DOF(q,dq,ddq,ZEW,modelParam);

%% Initialize description structure

constraintInfo.Inequalities = struct([]);
constraintInfo.Equalities = struct([]);

%% Inequality constraints (Add constraint 1-by-1 because of implementation)

% Initialize empty vector
C = [];

% COP conditions
% C = [C; (XCOP - XCOP_high)'];
% C = [C; (-XCOP + XCOP_low)'];
% % Add to description
% constraintInfo.Inequalities(2).Description = 'COPConditions';
% constraintInfo.Inequalities(2).Amount = 2*length(XCOP);


% Torque Limits
% C = [C; (GAMMA(1, :)' - modelParam.TorqueLimits(1))];
% C = [C; (-GAMMA(1, :)' - modelParam.TorqueLimits(1))];
% C = [C; (GAMMA(2, :)' - modelParam.TorqueLimits(2))];
% C = [C; (-GAMMA(2, :)' - modelParam.TorqueLimits(2))];
% C = [C; (GAMMA(3, :)' - modelParam.TorqueLimits(3))];
% C = [C; (-GAMMA(3, :)' - modelParam.TorqueLimits(3))];
% Add to description
% constraintInfo.Inequalities(3).Description = 'TorqueLimits';
% constraintInfo.Inequalities(3).Amount = 6*length(GAMMA(1, :));

%% Equality constraints

Ceq = [];
% Lift Off conditions ( along X and Y axis )
disp(size(Tlo{end, 1}))
disp(size(Tdo{end, 1}))
Ceq = [Ceq; Tlo{end, 1}(1, 4) - liftParam.WristPositionLiftOff(1)];
Ceq = [Ceq; Tlo{end, 1}(2, 4) - liftParam.WristPositionLiftOff(2)];

constraintInfo.Equalities(1).Description = 'WristPositionLiftOff';
constraintInfo.Equalities(1).Amount = 2;

% Drop Off conditions ( along X and Y axis )
Ceq = [Ceq; Tdo{end, 1}(1, 4) - liftParam.WristPositionDropOff(1)];
Ceq = [Ceq; Tdo{end, 1}(2, 4) - liftParam.WristPositionDropOff(2)];

constraintInfo.Equalities(2).Description = 'WristPositionDropOff';
constraintInfo.Equalities(2).Amount = 2;

end

