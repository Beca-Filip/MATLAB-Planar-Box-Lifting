%ANIMATE_COP

% Vertical gravitational force acting on the box COM, expressed in the 
% robot base frame
bF = [0; -LiftParam.BoxMass * LiftParam.Gravity; 0];

% Moment of the gravitational force with respect to the wrist, expressed in
% the robot base frame
bM = cross(-LiftParam.BoxToWristVectorDuringLift, bF);

% Get index of lift and drop off
iLiftOff = floor(size(q, 2) * LiftParam.PercentageLiftOff);
iDropOff = floor(size(q, 2) * LiftParam.PercentageDropOff);

% Get transformation matrices of the wrist joint during the lifting motion
T = FKM_nDOF_Cell(q(:, iLiftOff:iDropOff), L);
Tw = T(end, :);

% Get the force and moment of the gravitational force excerced on the wrist
% expressed in the wrist frame
wF = [];
wM = [];

for ii = 1 : length(Tw)
    % Add the rotated force vector to the data structure
    wF = [wF, Tw{ii}(1:3, 1:3)*bF];
    % Add the rotated moment vector to the data structure
    wM = [wM, Tw{ii}(1:3, 1:3)*bM];
end

% Get the zero external wrenches
EW = zeroExternalWrenches6DOF(size(q, 2));

% Modify the external wrenches at the end effector
EW.EndEffectorWrenches = [zeros(6, iLiftOff-1), [wF;wM], zeros(6, size(q, 2) - iDropOff)];

% Get COP
COP = COP_6DOF_Matrix(q_star, dq_star, ddq_star, modelParam, zeroExternalWrenches6DOF(size(q_star, 2)));
% Plot COP
opts.handleInits = {@()plot(0, 0, 'go', 'DisplayName', 'COP')};
opts.callback = @(ii, handle) cop_callback(ii, handle, COP);
% Plot COP bounds
opts.bgrPlot = @() bgr_plot(LiftParam.HeelPosition, LiftParam.ToePosition);

Animate_nDOF(q_star, L, Ts, opts);

function cop_callback(ii, handle, COP)
    handle.XData = COP(1, ii);
end

function bgr_plot(low, high)
    plot(low, 0, 'ro', 'DisplayName', 'CopLo');
    plot(high, 0, 'bo', 'DisplayName', 'CopHi')
end