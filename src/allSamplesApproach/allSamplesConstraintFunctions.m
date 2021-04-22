function [C, Ceq, constraintInfo] = allSamplesConstraintFunctions(x,timeParam,modelParam)
%ALLSAMPLESCONSTRAINTFUNCTIONS implements the constraints on a 3DOF
%model of a human performing a squat using all samples of the trajectory. 
%Returns the vector of values of the inequality and equality constraints, 
%as well as a structure containing the one word description of the 
%constraint and the number of elements corresponding to that constraint.

% Extract useful constraints
n = timeParam.NumSamples;
Ts = timeParam.SamplingPeriod;

% Unpack optimization variable
q1 = x(1:n);
q2 = x(n+1:2*n);
q3 = x(2*n+1:3*n);

%% Intermediate computations: Get joint trajectories, velocities and accelerations

% Trajectories
q_full = [q1; q2; q3];

% Velocities
dq_full = diff(q_full, 1, 2) / Ts;    % 1st order difference along second dimension

% Accelerations
ddq_full = diff(dq_full, 1, 2) / Ts;  % 1st order difference along second dimension

% Jerks
dddq = diff(ddq_full, 1, 2) / Ts;    % 1st order difference along second dimension

% Pick only the trajectory and velocity samples that will be used to
% calculate the cost function
q = q_full(:, 1:end-2);
dq = dq_full(:, 1:end-1);
ddq = ddq_full(:, 1:end);

% Get the number of samples that is going to take part in the calculation
% of the cost function
N = size(q, 2);

%% Intermediate Computations: Final State Neck and Hip Heights

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
ZEW = zeroExternalWrenches3DOF(N);

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

