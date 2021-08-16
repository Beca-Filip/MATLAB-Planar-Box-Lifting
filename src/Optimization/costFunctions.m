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
    L = [L, modelParam.(['L', num2str(ii)])];
end

% Normalized segment mass vector
M = [];
for ii = 1 : NJ
    M = [M, modelParam.(['M', num2str(ii)])];
end

% Segment centers of mass matrix
CMP = [];
for ii = 1 : NJ
    CMP = [CMP, modelParam.(['COM', num2str(ii)])];
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

%% Intermediate Computations: External Wrenches from box lifting

% % Find the time index where lift off is happening
% iLiftOff = round(Npts * liftParam.PercentageLiftOff);
% % Find the time index where drop off is happening
% iDropOff = round(Npts * liftParam.PercentageDropOff);
% 
% % Vertical gravitational force acting on the box COM, expressed in the 
% % robot base frame
% bF = [0; -liftParam.BoxMass * liftParam.Gravity; 0];
% 
% % Moment of the gravitational force with respect to the wrist, expressed in
% % the robot base frame
% bM = cross(-liftParam.BoxToWristVectorDuringLift, bF);
% 
% % Get transformation matrices of the wrist joint during the lifting motion
% T = FKM_nDOF_Cell(q(:, iLiftOff:iDropOff), L);
% Tw = T(end, :);
% 
% % Get the force and moment of the gravitational force excerced on the wrist
% % expressed in the wrist frame
% wF = [];
% wM = [];
% 
% for ii = 1 : length(Tw)
%     % Add the rotated force vector to the data structure
%     wF = [wF, -Tw{ii}(1:3, 1:3)*bF];
%     % Add the rotated moment vector to the data structure
%     wM = [wM, -Tw{ii}(1:3, 1:3)*bM];
% end
% 
% % Get the zero external wrenches
% EW = zeroExternalWrenches6DOF(size(q, 2));
% 
% % Modify the external wrenches at the end effector
% EW.EndEffectorWrenches = [zeros(6, iLiftOff-1), [wF;wM], zeros(6, size(q, 2) - iDropOff)];
EW = getExternalWrenches(q, L, liftParam);

%% Intermediate Computations: 

%% Intermediate Computations: Joint Torques

% Get wrenches from Dyn_3DOF
[GAMMA, ~] = Dyn_6DOF(q, dq, ddq, modelParam, EW);

% Get Joint Torque cost function
JT = sum(sum((GAMMA ./ modelParam.TorqueLimits.').^2, 2)) / itpParam.ItpResolutionCost / 3;

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
J = [J JJ];

%% Intermediate Computations: Joint Power

% Get Joint Power cost function
JP = sum(sum((dq .* GAMMA).^2, 2) ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% #Add to cost function vector
J = [J JP];

%% Intermediate Computations: Cartesian Velocity of the End-Effector

% Get the velocity
v_EE = FKM_Velocity_nDOF_Tensor(q, dq, L);

% Get the square of the end effector velocities
JEEV = sum(sum(v_EE.^2)) / itpParam.ItpResolutionCost / 2;

% #Add to cost function vector
J = [J JEEV];

%% Intermediate Computations: Cartesian Acceleration of the End-Effector

% Get the acceleration
a_EE = FKM_Acceleration_nDOF_Tensor(q, dq, ddq, L);

% Get the square of the end effector accelerations
JEEA = sum(sum(a_EE.^2)) / itpParam.ItpResolutionCost / 2;

% #Add to cost function vector
J = [J JEEA];

%% Intermediate Computations: Cartesian Velocity of the COM

% Get the velocity
v_COM = COM_Velocity_nDOF_Tensor(q, dq, L, M, CMP);

% Get the square of the center of mass velocities
JCOMV = sum(sum(v_COM.^2)) / itpParam.ItpResolutionCost / 2;

% #Add to cost function vector
J = [J JCOMV];

%% Intermediate Computations: Cartesian Acceleration of the COM

% Get the acceleration
a_COM = COM_Acceleration_nDOF_Tensor(q, dq, ddq, L, M, CMP);

% Get the square of the center of mass accelerations
JCOMA = sum(sum(a_COM.^2)) / itpParam.ItpResolutionCost / 2;

% #Add to cost function vector
J = [J JCOMA];

end

