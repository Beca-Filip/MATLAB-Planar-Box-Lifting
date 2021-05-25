clear all; close all; clc;

% Load data
load Subject1_Filip.mat

% Time vector
Ts = 0.01;
Time = 0:Ts:(size(q,2)-1)*Ts;

% Corrections to joint angles
q = centered_polar_angle(q);

%% Filter signals

% Steepness
qsteepness = 0.85;
fsteepness = 0.85;
msteepness = 0.85;
% Filter q
fpass = 5;
fs = 1 / Ts;
qf = lowpass(q.', fpass, fs, 'Steepness', 0.85).';

% Filter forceplate data
fnames = fieldnames(Forceplate);
for ii = 1 : length(fnames)
    % Filter
    Forceplate_filt.(fnames{ii}) = lowpass(Forceplate.(fnames{ii}), fpass, fs, 'Steepness', fsteepness);
end

% Filter Marker data
markersetnames = fieldnames(Markers);
for ii = 1 : length(markersetnames)
    markernames = fieldnames(Markers.(markersetnames{ii}));
    for jj = 1 : length(markernames)
        Markers.(markersetnames{ii}).(markernames{jj}) = lowpass(Markers.(markersetnames{ii}).(markernames{jj}), fpass, fs, 'Steepness', msteepness);
    end
end
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
        
        % Plot filtered joint profile
        plot(Time, qf(curr,:), 'DisplayName', [Joints{curr} ' filt']);
        
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
        
        % Plot filtered force profile
        plot(Time, Forceplate_filt.Forces(:,curr), 'DisplayName', [Forcenames{curr} ' filt']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Forcenames{curr} '-Force [N]']);
        title([Forcenames{curr} '-Force profile']);
        legend;
    end
end

%% Plot Moment Profiles

% Forceplate moment names
Momentnames = {'X', 'Y' 'Z'};

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
        
        % Plot moment profile
        plot(Time, Forceplate.Moments(:,curr), 'DisplayName', Momentnames{curr});
        
        % Plot filtered moment profile
        plot(Time, Forceplate_filt.Moments(:,curr), 'DisplayName', [Momentnames{curr} ' filt']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Momentnames{curr} '-Moment [Nm]']);
        title([Momentnames{curr} '-Moment profile']);
        legend;
    end
end

%% Plot COP Profiles

% Forceplate COP names
COPnames = {'X', 'Y' 'Z'};

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
        
        % Plot COP profile
        plot(Time, Forceplate.COP(:,curr), 'DisplayName', COPnames{curr}); 
        
        % Plot filtered moment profile
        plot(Time, Forceplate_filt.COP(:,curr), 'DisplayName', [COPnames{curr} ' filt']);
                
        % Labels
        xlabel('Time [s]');
        ylabel([COPnames{curr} '-COP [m]']);
        title([COPnames{curr} '-COP profile']);
        legend;
    end
end

%% Plot Height of each joint

% Add model functions to path
addpath('../../src/Model/');

% Segment lengths
L = [param.L1,param.L2,param.L3,param.L4,param.L5,param.L6];

% Get FKM for all joints
T = FKM_nDOF_Tensor(q, L);

% Get FKM for filtered joint angles
Tf = FKM_nDOF_Tensor(qf, L);

% Joint names
Joints = {'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow', 'Wrist'};

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
        
        % Plot joint height profiles
        plot(Time, squeeze(T(2, 4, curr+1,:)), 'DisplayName', Joints{curr});
        
        % Plot filtered joint height profile
        plot(Time, squeeze(Tf(2, 4, curr+1,:)), 'DisplayName', [Joints{curr} ' filt']);
        
        % Labels
        ylim([min(squeeze(T(2, 4, :, :)), [], 'all'), max(squeeze(T(2, 4, :, :)), [], 'all')]);
        xlabel('Time [s]');
        ylabel([Joints{curr} ' height [m]']);
        title([Joints{curr} ' height profile']);
        legend;
    end
end

% Get the segmented trajectory
handsegment = ginput;

% Get indices
segmentindices = floor(handsegment(:, 1) / Ts );

% Pause before animation
Animate_nDOF(q(:, segmentindices(1):segmentindices(2)), L, Ts);

%% Plot X-Position of every joint

% Joint names
Joints = {'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow', 'Wrist'};

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
        
        % Plot joint height profiles
        plot(Time, squeeze(T(1, 4, curr+1,:)), 'DisplayName', Joints{curr});
        
        % Plot filtered joint height profile
        plot(Time, squeeze(Tf(1, 4, curr+1,:)), 'DisplayName', [Joints{curr} ' filt']);
        
        % Labels
        ylim([min(squeeze(T(1, 4, :, :)), [], 'all'), max(squeeze(T(1, 4, :, :)), [], 'all')]);
        xlabel('Time [s]');
        ylabel([Joints{curr} ' forward comp [m]']);
        title([Joints{curr} ' forward component profile']);
        legend;
    end
end

%% Plot BOX and Table Markers height

% Names
tablemnames = fieldnames(Markers.TABLE);

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
        
        % Plot table marker height profiles
        plot(Time, Markers.TABLE.(tablemnames{curr})(:, 2), 'DisplayName', tablemnames{curr});
                
        % Labels
        xlabel('Time [s]');
        ylabel([tablemnames{curr} ' height [m]']);
        title(['Table marker heights']);
        legend;
    end
end


%% Save into new
q=qf;
Forceplate = Forceplate_filt;
save('Subject1_Filip_Clean.mat', 'q', 'Forceplate', 'Markers', 'param');

%%


function qo = centered_polar_angle(qi)
%CENTERED_POLAR_ANGLE brings all angles in range [-pi, pi).
    qo = mod(qi, 2*pi);
    qo(qo >= pi) = qo(qo >= pi) - 2*pi;
end