function [J] = costFunctions(x,itpParam,modelParam,liftParam)
%COSTFUNCTIONS implements the desired cost function of a 6DOF model of a 
%human performing a box-lifting task.

% Extract useful constants
Ncp = itpParam.NumControlPoints;
Tknots = itpParam.KnotValues;
ItpOrder = itpParam.InterpolationOrder;
BndCnd = itpParam.BoundaryConditions;

% Interpolation for constraints
Npts = itpParam.ItpResolutionCost;
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

% Initialize cost function vector
J = [];

%% Intermediate Computations: Spline Interpolation

% Get spline coeffs
polycoeffs = cell(1, NJ);
for ii = 1 : NJ
    polycoeffs{ii}  = splineInterpolation2(Tknots, q_knots{ii}, ItpOrder, BndCnd);
end

% Get trajectories, velocities, accelerations, and jerks and separate them
q_tva = cell(1, NJ);
cell_q = cell(1, NJ);
cell_dq = cell(1, NJ);
cell_ddq = cell(1, NJ);
cell_dddq = cell(1, NJ);

for ii = 1 : NJ
    q_tva{ii} = splineCoefToTrajectory(Tknots, polycoeffs{ii}, Time, 3);
    cell_q{ii} = q_tva{ii}(1, :).'; % Transpose for later
    cell_dq{ii} = q_tva{ii}(2, :).'; % Transpose for later
    cell_ddq{ii} = q_tva{ii}(3, :).'; % Transpose for later 
    cell_dddq{ii} = q_tva{ii}(4, :).'; % Transpose for later    
end

% Merge trajectories velocities and accelerations into one matrix
q = [cell_q{:}].'; % Merge and transpose so rows correspond to single joint trajectories
dq = [cell_dq{:}].'; % Merge and transpose so rows correspond to single joint velocities
ddq = [cell_ddq{:}].'; % Merge and transpose so rows correspond to single joint accelerations
dddq = [cell_dddq{:}].'; % Merge and transpose so rows correspond to single joint jerks

%% Intermediate Computations: Joint Torques

% % Generate zero external wrenches structure as required by Dyn_3DOF
% ZEW = zeroExternalWrenches3DOF(itpParam.ItpResolutionCost);
% 
% % Get wrenches from Dyn_3DOF
% [GAMMA, ~] = Dyn_3DOF(q, dq, ddq, ZEW, modelParam);
% 
% % Get Joint Torque cost function
% JT = sum(sum(GAMMA.^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;
% 
% % #Add to cost function vector
% J = [J JT];

%% Intermediate Computations: Joint Acceleration

% Get Joint Acceleration cost function
JA = sum(sum(ddq.^2)) / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
J = [J JA];

%% Intermediate Computations: Joint Jerk

% Get Joint Jerk cost function
% JJ = sum(sum(dddq.^2)) / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
% J = [J JJ];

%% Intermediate Computations: Joint Power

% Get Joint Power cost function
% JP = sum(sum((dq .* GAMMA).^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
% J = [J JP];
end

