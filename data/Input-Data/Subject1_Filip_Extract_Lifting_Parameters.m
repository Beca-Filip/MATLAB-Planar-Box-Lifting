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

% Cutoff the trajectory at drop off
q = q(:, 1:iDropOff);
Time = Time(1:iDropOff);
Forceplate.Forces = Forceplate.Forces(1:iDropOff, :);
Forceplate.Moments = Forceplate.Moments(1:iDropOff, :);
Forceplate.COP = Forceplate.COP(1:iDropOff, :);
for ii = 1 : length(msetnames)
    mnames = fieldnames(Markers.(msetnames{ii}));    
    for jj = 1 : length(mnames)
        Markers.(msetnames{ii}).(mnames{jj})  = Markers.(msetnames{ii}).(mnames{jj})(1:iDropOff, :);
    end
end

%% Filter the joint angles

% Filter input data
freq_lim = 2;
% q = lowpass_filter(q, 1/Ts, freq_lim, 5);    

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
% plot([Time(iDropOff) Time(iDropOff)], mm, '--', 'DisplayName', 'DropOff', 'Color', [0.8 0 0.8]);
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


%% Draw the box and the table

% Parameters related to the table
TableUpperNearCorner = mean(Markers.TABLE.NEAR - Markers.BODY.ANKLE);
TableUpperFarCorner = mean(Markers.TABLE.FARR - Markers.BODY.ANKLE);
TableLowerNearCorner = TableUpperNearCorner;
TableLowerNearCorner(2) = 0;

% Table Params
LiftParam.TableWidth = TableUpperFarCorner(1) - TableUpperNearCorner(1);
LiftParam.TableHeight = TableUpperNearCorner(2)-0.02;
LiftParam.TableLowerNearCorner = TableLowerNearCorner;

% Lower left corner x-y coordinates, then width and height
TableRectangle=[TableLowerNearCorner(1:2), LiftParam.TableWidth, LiftParam.TableHeight];
LiftParam.TableRectangle=TableRectangle;

% Box Parameters (From AMAZON product info)
LiftParam.BoxWidth = 0.33;    % 33cm
LiftParam.BoxHeight = 0.248;  % 24,8 cm

% Box corners
BoxLowerFarCorner = Markers.BOX.FARR - Markers.BODY.ANKLE;
BoxLowerNearCorner = BoxLowerFarCorner;
BoxLowerNearCorner(:, 1) = BoxLowerNearCorner(:, 1) - LiftParam.BoxWidth;

% Box COM
BoxCOM = (BoxLowerNearCorner + BoxLowerFarCorner ) / 2 ;
% Box to wrist vector
LiftParam.BoxToWristVectorDuringLift = mean((Markers.BODY.WRIST(iLiftOff:iDropOff, :) - Markers.BODY.ANKLE(iLiftOff:iDropOff, :)) - BoxCOM(iLiftOff:iDropOff, :)).';
% Box Rectangle
BoxLowerNearCorner = BoxLowerNearCorner(1, :);
BoxLowerFarCorner = BoxLowerFarCorner(1, :);
LiftParam.BoxRectangleInitial = [BoxLowerNearCorner(1:2), LiftParam.BoxWidth, LiftParam.BoxHeight];
% Box Rectangle from box COM
LiftParam.BoxRectangle=@(BoxCOM)[BoxCOM(:, 1:2)-LiftParam.BoxWidth/2, LiftParam.BoxWidth, LiftParam.BoxHeight];
% Animation options
% opts.bgrPlot = @()background(TableRectangle, Twlo); 

opts.handleInits = {@()handle_inits(LiftParam)};
opts.callback = @(ii, handle) animate_callbacks(ii, handle, iLiftOff, iDropOff, diff(Markers.BODY.WRIST - Markers.BODY.WRIST(iLiftOff, :)));
% Animate
Animate_nDOF(q, L, Ts, opts);

%% Parameters related to the box
% Box mass
LiftParam.BoxMass = 10.5;
% Gravity
LiftParam.Gravity = 9.81;

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
LiftParam.FinalAngles = q(:, end);


opts.bgrPlot = @()background(TableRectangle, Twlo); 
Animate_nDOF(q, L, Ts, opts);


%% Plot all joint torque profiles

addpath('../../src\OptimizatiLifon');
EW = getExternalWrenches(q, L, LiftParam);

% Get torques
dq = diff(q, 1, 2) / Ts;
ddq = diff(q, 2, 2) / Ts^2;
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

%% COP related parameters

% Get COP 
COP = COP_6DOF_Matrix(q,dq,ddq,param,EW);

% Toe and heel positions
LiftParam.HeelPosition = mean(Markers.BODY.HEEL(:, 1)-Markers.BODY.ANKLE(:, 1)) - 0.02; % Remove 2cm
LiftParam.ToePosition  = mean(Markers.BODY.METATARSAL(:, 1)-Markers.BODY.ANKLE(:, 1)) + 0.13; % Add 13cm since marker was at 2/3 of the foot length'
% LiftParam.ToePosition = max(COP(1, :)) + 0.01;  % Add 1cm

%%
% Cutoff the trajectory at Lift off
q = q(:, 1:iLiftOff);
Time = Time(1:iLiftOff);
Forceplate.Forces = Forceplate.Forces(1:iLiftOff, :);
Forceplate.Moments = Forceplate.Moments(1:iLiftOff, :);
Forceplate.COP = Forceplate.COP(1:iLiftOff, :);
for ii = 1 : length(msetnames)
    mnames = fieldnames(Markers.(msetnames{ii}));    
    for jj = 1 : length(mnames)
        Markers.(msetnames{ii}).(mnames{jj})  = Markers.(msetnames{ii}).(mnames{jj})(1:iLiftOff, :);
    end
end
% Lifting parameters (lift off and drop off)
LiftParam.PercentageLiftOff = iLiftOff / size(q, 2);
LiftParam.PercentageDropOff = iDropOff / size(q, 2);

% Save
modelParam = param;

% Add number of joints
modelParam.NJoints = 6;

% Add joint limits
[lb, ub] = expandIntervalByPercentage(10, min(q, [], 2).', max(q, [], 2).');
modelParam.JointLimits = [lb; ub];

% Add torque limits (TAKEN FROM ROBERT 2013)

maxTorque_ANKLE = 2*126; % Double because of symmetry
maxTorque_KNEE = 2*100; % Double because of symmetry
maxTorque_HIP = 2*185; % Double because of symmetry
maxTorque_BACK = 143;   % LUMBAR JOINT in the paper
maxTorque_SHOULDER = 2*92;    % Double because of symmetry
maxTorque_ELBOW = 2*77;     % Double because of symmetry

modelParam.TorqueLimits = [maxTorque_ANKLE, maxTorque_KNEE, maxTorque_HIP, maxTorque_BACK, maxTorque_SHOULDER, maxTorque_ELBOW];

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

% function background(TableRectangle, BoxRectangle)
function background(TableRectangle, Twlo)
rectangle('Position', TableRectangle, 'EdgeColor', [0 0 0]);
% rectangle('Position', BoxRectangle, 'EdgeColor', [0 0 1]);
plot(Twlo(1, 4, end), Twlo(2, 4, end), 'ro', 'DisplayName', 'Twlo');
end

function h = handle_inits(LiftParam)
    h = rectangle('Position', LiftParam.BoxRectangleInitial);
end

function animate_callbacks(ii, handle, iLiftOff, iDropOff, wristdiff)
    if ii > iLiftOff && ii <= iDropOff
        handle.Position(1:2) = handle.Position(1:2) + wristdiff(ii-1, 1:2);
%         handle.Position(1:2) = LiftParam.BoxRectangle(1:2) + wristdiff(ii, 1:2);
    end
end