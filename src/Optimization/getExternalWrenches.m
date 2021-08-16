function [EW] = getExternalWrenches(q,L,LiftParam)
%GETEXTERNALWRENCHES blablabla. Gets external wrenches.
%   
%   EW = GETEXTERNALWRENCHES(q, L, LiftParam)

% If we're only looking up to the lift off
if LiftParam.PercentageLiftOff >= 1
    % Get the zero external wrenches
    EW = zeroExternalWrenches6DOF(size(q, 2));
    return
end

% Get indices of the beginning and ending of lift
iLiftOff = floor(LiftParam.PercentageLiftOff * size(q, 2));
iDropOff = floor(LiftParam.PercentageDropOff * size(q, 2));

% Vertical gravitational force acting on the box COM, expressed in the 
% robot base frame
bF = [0; -LiftParam.BoxMass * LiftParam.Gravity; 0];

% Moment of the gravitational force with respect to the wrist, expressed in
% the robot base frame
bM = cross(-LiftParam.BoxToWristVectorDuringLift, bF);

% Get transformation matrices of the wrist joint during the lifting motion
T = FKM_nDOF_Cell(q(:, iLiftOff:iDropOff), L);
Tw = T(end, :);

% Get the force and moment of the gravitational force excerced on the wrist
% expressed in the wrist frame
wF = [];
wM = [];

for ii = 1 : length(Tw)
    % Add the rotated force vector to the data structure
    wF = [wF, -Tw{ii}(1:3, 1:3).'*bF];
    % Add the rotated moment vector to the data structure
    wM = [wM, -Tw{ii}(1:3, 1:3).'*bM];
end

% Get the zero external wrenches
EW = zeroExternalWrenches6DOF(size(q, 2));

% Modify the external wrenches at the end effector
EW.EndEffectorWrenches = [zeros(6, iLiftOff-1), [wF;wM], zeros(6, size(q, 2) - iDropOff)];
end

