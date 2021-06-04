clear all; close all; clc;

% Add model functions to path
addpath('../../src/Model/');

% Load clean data
load Subject1_Filip_Clean.mat

% Extract forces
fnames = fieldnames(Forceplate);
for ii = 1 : length(fnames)
    Forceplate.(fnames{ii}) = Forceplate.(fnames{ii})(segmentindices(1):segmentindices(2), :);
end

% Extract markers
msetnames = fieldnames(Markers);
for ii = 1 : length(msetnames)
    mnames = fieldnames(Markers.(msetnames{ii}));    
    for jj = 1 : length(mnames)
        Markers.(msetnames{ii}).(mnames{jj})  = Markers.(msetnames{ii}).(mnames{jj})(segmentindices(1):segmentindices(2), :);
    end
end

% Extract angles
q = q(:, segmentindices(1):segmentindices(2));

% Time vector
Ts = 0.01;
Time = 0:Ts:(size(q,2)-1)*Ts;

% Get lengths vector
L = [param.L1,param.L2,param.L3,param.L4,param.L5,param.L6];

% Manually found indices of liftoff
iLiftOff = 348;
iDropOff = 526;
%% Plot all joint profiles

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% How many rows
figcols = 2;
figrows = 3;

figure;

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, q(curr,:), 'DisplayName', Joints{curr});
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angle [rad]']);
        title([Joints{curr} ' joint trajectory']);
        legend;
    end
end

%% Plot Force Profiles

% Forceplate names
Forcenames = {'X', 'Y' 'Z'};

% How many rows
figcols = 1;
figrows = 3;

figure;

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot force profile
        plot(Time, Forceplate.Forces(:,curr), 'DisplayName', Forcenames{curr});

        % Labels
        xlabel('Time [s]');
        ylabel([Forcenames{curr} '-Force [N]']);
        title([Forcenames{curr} '-Force profile']);
        legend;
    end
end


%% Plot Z force and height of box

% How many rows
figcols = 1;
figrows = 2;

figure;

yyaxis left;
plot(Time, Forceplate.Forces(:,3), 'DisplayName', Forcenames{3});
ylabel([Forcenames{3} '-Force [N]']);
yyaxis right;
hold on;
plot(Time, Markers.BOX.FARR(:, 2), 'DisplayName', 'BoxHeight');
mm = [min(Markers.BOX.FARR(:, 2)), max(Markers.BOX.FARR(:, 2))];
plot([Time(iLiftOff) Time(iLiftOff)], mm, '--', 'DisplayName', 'LiftOff', 'Color', [1 0 1]);
plot([Time(iDropOff) Time(iDropOff)], mm, '--', 'DisplayName', 'DropOff', 'Color', [0.8 0 0.8]);
ylabel(['Height [m]']);
legend;
title('Box height and vertical force profiles');

% %% Get graphic input
% ptimes = ginput;
% 
% % Indices
% si = floor(ptimes(:, 1) / Ts);
% % Animate
% Animate_nDOF(q(:, si(1):si(2)), L, Ts);


%% Save the data and export lifting parameters

% Lifting parameters (lift off and drop off)
LiftParam.PercentageLiftOff = iLiftOff / size(q, 2);
LiftParam.PercentageDropOff = iDropOff / size(q, 2);

% Get cartesian position of wrists for lift off
Twlo = FKM_nDOF_Tensor(q(:, iLiftOff), L);
Twdo = FKM_nDOF_Tensor(q(:, iDropOff), L);

LiftParam.WristPositionLiftOff = Twlo(1:3, 4, end);
LiftParam.WristPositionDropOff = Twdo(1:3, 4, end);
LiftParam.InitialAngles = q(:, 1);
LiftParam.FinalAngles = q(:, 2);

% Toe and heel positions
LiftParam.HeelPosition = mean(Markers.BODY.HEEL(:, 1)-Markers.BODY.ANKLE(:, 1));
LiftParam.ToePosition  = mean(Markers.BODY.METATARSAL(:, 1)-Markers.BODY.ANKLE(:, 1)) + 0.1; % Add 10cm since marker was at 2/3 of the foot length'

%% Parameters related to the box
% Box mass
LiftParam.BoxMass = 10.5;
% Gravity
LiftParam.Gravity = 9.81;

% Center of the box from markers
BoxCOM = (Markers.BOX.FARR + Markers.BOX.NEAR) / 2;
% Position vector from the box to the wrist accross time
BoxToWristVectorDuringLift = Markers.BODY.WRIST(iLiftOff:iDropOff, :) - BoxCOM(iLiftOff:iDropOff, :);
% Mean of the position vector (Make a column vector)
LiftParam.BoxToWristVectorDuringLift = mean(BoxToWristVectorDuringLift).';

%% Plot all joint torque profiles

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

% Get torques
dq = diff(q, 1, 2);
ddq = diff(q, 2, 2);
dq = [dq dq(:, end)];
ddq = [ddq ddq(:, end-1:end)];
[GAMMA, ~] = Dyn_6DOF(q, dq, ddq, param, EW);

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% How many rows
figcols = 2;
figrows = 3;

figure;

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, GAMMA(curr,:), 'DisplayName', Joints{curr});
        
        mm = [min(GAMMA(curr, :)), max(GAMMA(curr, :))];
        plot([Time(iLiftOff) Time(iLiftOff)], mm, '--', 'DisplayName', 'LiftOff', 'Color', [1 0 1]);
        plot([Time(iDropOff) Time(iDropOff)], mm, '--', 'DisplayName', 'DropOff', 'Color', [0.8 0 0.8]);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' torque [Nm]']);
        title([Joints{curr} ' torque profile']);
        legend;
    end
end


%%
% Save
modelParam = param;
% Add joint limits
[lb, ub] = expandIntervalByPercentage(10, min(q, [], 2).', max(q, [], 2).');
modelParam.JointLimits = [lb; ub];
% Add torque limits
[lb, ub] = expandIntervalByPercentage(20, min(GAMMA, [], 2).', max(GAMMA, [], 2).');
modelParam.TorqueLimits = max(abs([lb;ub]));
% save('Subject1_Filip_Segmented.mat', 'modelParam', 'q', 'Markers', 'Forceplate', 'LiftParam');

%% 
function [lb, ub] = expandIntervalByPercentage(pc, lb, ub)
    % get interval width
    interval_width = ub - lb;
    % if width <0 fix at 0
    interval_width(interval_width < 0) = 0;
    
    % percentage to multiplier
    mul = pc / 100;
    
    % add to ub and substract from lb
    lb = lb - mul .* interval_width / 2;
    ub = ub + mul .* interval_width / 2;
end