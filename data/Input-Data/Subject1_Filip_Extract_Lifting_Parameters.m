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
LiftParam.HeelPosition = mean(Markers.BODY.HEEL(:, 1));
LiftParam.ToePosition  = mean(Markers.BODY.METATARSAL(:, 1)) + 0.1; % Add 10cm since marker was at 2/3 of the foot length

%% Plot COP

COP = COP_6DOF_Matrix(q,dq,ddq,)

%%
% Save
modelParam = param;
% Add limits
modelParam.JointLimits = [min(q, [], 2).'; max(q, [], 2).'];

save('Subject1_Filip_Segmented.mat', 'modelParam', 'q', 'Markers', 'Forceplate', 'LiftParam');