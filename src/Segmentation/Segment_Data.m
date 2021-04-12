%SEGMENT_DATA Implements an algorithm to segment a collection of squatting
%movements into individual squatting movements based on the heights of the
%right and left shoulder markers, their mean value, as well as their
%maximum and minimum values. The result is a rough segmentation of the
%squatting motions.
%The final segmentation is done by hand using the ginput function. The
%times and corresponding sample indices are saved in beginning-end pairs
%within two matrices:TIMES and INDICES, which are in turn saved in the
%SquatTimeSegments.mat

clear all; close all; clc;

%% Import 
% Add the data to path
addpath("../../data/3DOF/Squat");
% Load model data
load squat_param.mat
load Kinematics_meas_Sb4_Tr2.mat

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");


%% Plot time series & Locate crunch positions
Ts = 0.01;  % 10 ms - f = 100Hz
time = 0:Ts:(length(Markers.RSHO) - 1)*Ts;

% Locate minima
minLogIndR = islocalmin(Markers.RSHO(:, 2)) & (Markers.RSHO(:, 2) < mean(Markers.RSHO(:, 2)));
minLogIndL = islocalmin(Markers.LSHO(:, 2)) & (Markers.LSHO(:, 2) < mean(Markers.LSHO(:, 2)));
minIndR = find(minLogIndR);
minIndL = find(minLogIndL);

% Compress comments that were used to build filterMinima function
for CompressComments = 1:1
% 
% %% Get the time differences between each successive minima
% timeDiffR = diff(time(minIndR));
% timeDiffL = diff(time(minIndL));
% 
% % Get mean time difference
% meanTimeDiffR = mean(timeDiffR);
% meanTimeDiffL = mean(timeDiffL);
% 
% % Get the indices where the time difference is lower than the mean
% smallTimeDiffR = find(timeDiffR < meanTimeDiffR);
% smallTimeDiffL = find(timeDiffL < meanTimeDiffL);

% %% Those indices represent the location of doubles
% toRemove = [];
% for ii = 1 : length(smallTimeDiffR)
%     
%     % Indices of minima that are doubles
%     dm1 = (smallTimeDiffR(ii)+1);
%     dm2 = (smallTimeDiffR(ii));
%     
%     % Indices of the samples that are doubles
%     ds1 = minIndR(dm1);
%     ds2 = minIndR(dm2);
%     
%     % Values of samples that are doubles
%     dv1 = Markers.RSHO(ds1, 2);
%     dv2 = Markers.RSHO(ds2, 2);
%     
%     % Compare values and remove the larger
%     if dv1 <= dv2
%         toRemove = [toRemove dm2];
%     else
%         toRemove = [toRemove dm1];
%     end
% end
% 
% % Remove elements
% minIndR(toRemove) = [];
% 
% % Those indices represent the location of doubles
% toRemove = [];
% for ii = 1 : length(smallTimeDiffL)
%     
%     % Indices of minima that are doubles
%     dm1 = (smallTimeDiffL(ii)+1);
%     dm2 = (smallTimeDiffL(ii));
%     
%     % Indices of the samples that are doubles
%     ds1 = minIndL(dm1);
%     ds2 = minIndL(dm2);
%     
%     % Values of samples that are doubles
%     dv1 = Markers.LSHO(ds1, 2);
%     dv2 = Markers.LSHO(ds2, 2);
%     
%     % Compare values and remove the larger
%     if dv1 <= dv2
%         toRemove = [toRemove dm2];
%     else
%         toRemove = [toRemove dm1];
%     end
% end
% 
% % Remove elements
% minIndL(toRemove) = [];
end

% Filter minima by removing those that are too close together
minIndR = filterMinima(minIndR, time, Markers.RSHO(:, 2), inf);
minIndL = filterMinima(minIndL, time, Markers.LSHO(:, 2), inf);

% Get maximal value
[maxR] = max(Markers.RSHO(:, 2));
[minR] = min(Markers.RSHO(:, 2));
[maxL] = max(Markers.LSHO(:, 2));
[minL] = min(Markers.LSHO(:, 2));
maxminR = .95 * (maxR - minR);
maxminL = .95 * (maxL - minL);

figure;

hold on;
plot(time, Markers.RSHO(:, 2), 'DisplayName', 'y-RSHO');
plot(time(minIndR), Markers.RSHO(minIndR, 2), 'bo', 'HandleVisibility', 'Off');
plot(time, Markers.LSHO(:, 2), 'DisplayName', 'y-LSHO');
plot(time(minIndL), Markers.LSHO(minIndL, 2), 'ro', 'HandleVisibility', 'Off');
plot(time(1:end-1:end), repmat(minR, 1, 2) + maxminR, 'Color', [.5 .5 0], 'DisplayName', 'MaxValueR');
plot(time(1:end-1:end), repmat(minL, 1, 2) + maxminL, 'Color', [.5 .5 0], 'DisplayName', 'MaxValueL');
title('Shoulder markers height across time');
xlabel('Time [s]');
ylabel('Height [m]');
grid;
legend('Location', 'Best');

% Look at velocity
GradRSHO = diff(Markers.RSHO(:, 2)) / Ts;
GradLSHO = diff(Markers.LSHO(:, 2)) / Ts;

figure;

hold on;
plot(time(1:end-1)', GradRSHO, 'DisplayName', 'grad-y-RSHO');
plot(time(minIndR)', GradRSHO(minIndR), 'bo', 'DisplayName', 'min-y-RSHO');
plot(time(1:end-1)', GradLSHO, 'DisplayName', 'grad-y-LSHO');
plot(time(minIndL)', GradLSHO(minIndL), 'ro', 'DisplayName', 'min-y-LSHO');
title('Differential of should height across time');
xlabel('Time [s]');
ylabel('Height [m]');
grid;
legend('Location', 'Best');

for CompressComments = 1:1
% % Look at acceleration
% HessRSHO = diff(GradRSHO) / Ts;
% HessLSHO = diff(GradLSHO) / Ts;
% 
% figure;
% 
% hold on;
% plot(time(1:end-2)', HessRSHO, 'DisplayName', 'grad-y-RSHO');
% plot(time(minIndR)', HessRSHO(minIndR), 'bo', 'DisplayName', 'min-y-RSHO');
% plot(time(1:end-2)', HessLSHO, 'DisplayName', 'grad-y-LSHO');
% plot(time(minIndL)', HessLSHO(minIndL), 'ro', 'DisplayName', 'min-y-LSHO');
% title('Differential of should height across time');
% xlabel('Time [s]');
% ylabel('Height [m]');
% grid;
% legend('Location', 'Best');
end

%% Segment trajectory

% Length of data
N = length(Markers.RSHO(:, 2));
% Trajectory windows
widnows = zeros(2, length(minIndR));
% Filter for finding maximum: size
max_filtsize = 1;

% For each minima
for ii = 1 : length(minIndR)
    
    % Go to the left and right of the minima
    cntLeft = minIndR(ii);
    cntRight = minIndR(ii);
    
    % Left size
    while cntLeft > 1 && ~stopSearch(cntLeft, Markers.RSHO(:, 2), minR+maxminR, max_filtsize)
        cntLeft = cntLeft - 1;
    end
    
    % Right side
    while cntRight < N && ~stopSearch(cntRight, Markers.RSHO(:, 2), minR+maxminR, max_filtsize)
        cntRight = cntRight + 1;
    end
    
    % Store the windows
    windows(1, ii) = cntLeft;
    windows(2, ii) = cntRight;
end

% Get segmented trajectories
st = segmentedTrajectories(windows, time, Markers.RSHO(:, 2));

% Plot segmented trajectories
colorDist = @(flag, ratio) [ratio 1-ratio] .* [flag 1-flag];
figure;
hold on;
plot(time, Markers.RSHO(:, 2), 'DisplayName', 'OG Traj');
for ii = 1 : length(st)
    Rcomp = 1 * mod(ii, 2);
    Gcomp = 1 * mod(ii+1, 2);
    plot(st{ii}(1, :), st{ii}(2, :), 'Color', [Rcomp Gcomp 0], 'LineWidth', 2, 'DisplayName', ['Seg_{' num2str(ii) '}']);
end

title('Segmented trajectory of should height across time');
xlabel('Time [s]');
ylabel('Height [m]');
grid;
legend('Location', 'Best');

%% Segment by hand
% Change directory for saves
cd('../../data/3DOF/');

% Make directory if it doesn't already exist
if ~exist('Segmentation', 'file')
    mkdir Segmentation
end

% Enter directory
cd('Segmentation');

% If a save already exists
if exist('SquatTimeSegments.mat', 'file')
    answ = input('Do you want to override current segmentation (Y - yes, otherwise no):\n', 's');
end

% If a save doesn't exist or the answer is Yes

if ~exist('SquatTimeSegments.mat', 'file') || answ == 'Y'
    % Get new segmentation
    SEGMENTS = ginput;

    % Extract time
    TIME = SEGMENTS(:, 1);

    % For each time look for corresponding index
    INDEX = zeros(size(TIME));
    for ii = 1 : length(TIME)
        INDEX(ii) = find(time > TIME(ii), 1, 'first');
    end

    % Pair the time and index segments
    TIMES = [TIME(1:2:end), TIME(2:2:end)];
    INDICES = [INDEX(1:2:end), INDEX(2:2:end)];
    MINIMA_INDICES = minIndL(2:end);

% If no override load previous save
else
    load SquatTimeSegments.mat
end

% Save
save('SquatTimeSegments.mat', 'TIMES', 'INDICES', 'MINIMA_INDICES');

% Go back to original directory
cd(strrep(mfilename('fullpath'), mfilename, ''));

%% Functions

function st = segmentedTrajectories(windows, time, data)
%SEGMENTEDTRAJECTORIES returns the pairs of time-data trajectories between
%pairs of indices given in windows.

    % Prealocate output
    st = cell(size(windows, 2), 1);
    
    % For each pair of indices
    for ii = 1 : size(windows, 2)
       
        % St
        st{ii}(1, :) = time(windows(1, ii) : windows(2, ii));
        st{ii}(2, :) = data(windows(1, ii) : windows(2, ii))';
        
    end
end

function indMin = filterMinima(indMin, time, data, meanTimeDiff)
%FILTERMINIMA takes in the indices of the minima, and the time and data 
%to which the minima correspond. Returns the indices of the minima which
%are to be kept.

    % Get the difference of minima time
    timeDiff = diff(time(indMin));
    
    % Get the mean of the time differences
    if meanTimeDiff > mean(timeDiff)
        meanTimeDiff = mean(timeDiff);
    end
    
    % Get the minima that are too close together
    smallTimeDiff = find(timeDiff < meanTimeDiff);
    
    % Remove the elements of the minima that are too close together
    % Those indices represent the location of doubles
    toRemove = [];
    for ii = 1 : length(smallTimeDiff)

        % Indices of minima that are doubles
        dm1 = (smallTimeDiff(ii)+1);
        dm2 = (smallTimeDiff(ii));

        % Indices of the samples that are doubles
        ds1 = indMin(dm1);
        ds2 = indMin(dm2);

        % Values of samples that are doubles
        dv1 = data(ds1);
        dv2 = data(ds2);

        % Compare values and remove the larger
        if dv1 <= dv2
            toRemove = [toRemove dm2];
        else
            toRemove = [toRemove dm1];
        end
    end
    
    % If none to remove return the current index vector
    if isempty(toRemove)
        return
    % If some are to be removed, remove them and re-filter minima
    else
        indMin(toRemove) = [];
        indMin = filterMinima(indMin, time, data, meanTimeDiff);
    end
end

function flag = stopSearch(cnt, data, maxmin, max_filtsize)
    % If data is less than percent of peak to peak value continue search
    if data(cnt) < maxmin
        flag = false;
        return
    end
    
    % Else if data is bigger, take it if it is a maximum in a local area of
    % given size
    size = max_filtsize;
    
    % First check that cnt is within bounds
    N = length(data);
    
    % Lower bound check ( if cnt - size is under lower bound )
    if cnt - size < 1 
        % Left bound
        sizeL = cnt - 1;
    else
        % left bound is of size
        sizeL = size;
    end
    
    % Upperbound check
    if cnt + size > N
        % Upper bound
        sizeR = N - cnt;
    else
        % right bound is of size
        sizeR = size;
    end
    
    % Get part of data of size 2*size around cnt
    window = data(cnt-sizeL : cnt+sizeR);
    
    % Maximum of window
    m = max(window);
    
    % If our sample is the max stop the search
    if data(cnt) == m
        flag = true;
        return
    % Else continue
    else
        flag = false;
        return
    end
end
