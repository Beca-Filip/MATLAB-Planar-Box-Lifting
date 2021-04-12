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

%%
% Get all field names
fn = fieldnames(Markers);

% Relevant field names
rn = {};
ln = {};
on = {};

% Init RDATA and LDATA and ODATA
RDATA = [];
LDATA = [];
ODATA = [];

% Search through them
for ii = 1 : numel(fn)
    % For fieldnames corresponding to markers on the right side (having
    % first character 'R')
    if fn{ii}(1) == 'R'
        % Add those field names to relevant field names
        rn{end+1} = fn{ii};
                
        % Get the current marker's data
        RDATA = cat(3, RDATA, Markers.(fn{ii}));
    end

    % For fieldnames corresponding to markers on the left side (having
    % the first character be 'L')
    if fn{ii}(1) == 'L'
        % Add those field names to relevant field names
        ln{end+1} = fn{ii};
                
        % Get the current marker's data
        LDATA = cat(3, LDATA, Markers.(fn{ii}));
    end
    
    % For fieldnames not corresponding to left or right markers
    if fn{ii}(1) ~= 'L' && fn{ii}(1) ~= 'R'
        % Add those field names to a list
        on{end+1} = fn{ii};
        
        % Get the current marker's data
        ODATA = cat(3, ODATA, Markers.(fn{ii}));
    end
end

% Remove RANKproj
RDATA = RDATA(:, :, 1:end-1);
rn = rn(1:end-1);

% Stack data
DATA = cat(3, RDATA, LDATA);

% Permute dimensions of RDATA and LDATA
RDATA = permute(RDATA, [2 1 3]);
LDATA = permute(LDATA, [2 1 3]);
ODATA = permute(ODATA, [2 1 3]);

% Order
Order = [9 8 7 6 5 4 3 1];

% Reorder
disp(rn(Order));
disp(ln(Order));

rn = rn(Order);
ln = ln(Order);

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

% For each minima
for ii = 1 : length(minIndR)
    
    % Go to the left and right of the minima
    cntLeft = minIndR(ii);
    cntRight = minIndR(ii);
    
    % Left size
    while cntLeft > 1 && ~stopSearch(cntLeft, Markers.RSHO(:, 2), minR+maxminR)
        cntLeft = cntLeft - 1;
    end
    
    % Right side
    while cntRight < N && ~stopSearch(cntRight, Markers.RSHO(:, 2), minR+maxminR)
        cntRight = cntRight + 1;
    end
    
    % Store the windows
    windows(1, ii) = cntLeft;
    windows(2, ii) = cntRight;
end

% 

%% Prepare Animation

% Sampling time
Ts = 0.01;
% Num samples
N = length(Markers.RANK);

% Preload figure and handles
figure;
hold all;
handleL = plot3([0], [0], [0], 'ro', [0], [0], [0], 'DisplayName', 'LMarkers');
handleR = plot3([0], [0], [0], 'bo', [0], [0], [0], 'DisplayName', 'RMarkers');
handleO = plot3([0], [0], [0], 'go', 'DisplayName', 'OMarkers');
xlim([min(squeeze(DATA(:, 1, :)), [], 'all') max(squeeze(DATA(:, 1, :)), [], 'all')]);
ylim([min(squeeze(DATA(:, 2, :)), [], 'all') max(squeeze(DATA(:, 2, :)), [], 'all')]);
zlim([min(squeeze(DATA(:, 3, :)), [], 'all') max(squeeze(DATA(:, 3, :)), [], 'all')]);
xlabel('X-axis [m]');
ylabel('Y-axis [m]');
zlabel('Z-axis [m]');
grid;
% legend;
set(gca,'DataAspectRatio',[1 1 1])
% view(90, 50);
% view(-121.6899,-74.1249);
% view(-121.6899,30);

% Create a timer to repeat the update of the plot handles
% Initialize it
t = timer;
% Create an starting function
t.StartFcn = @(~,thisEvent)fprintf("Animation running...\n");
% Create a callback function
t.TimerFcn = @(thisTimer, ~)animateCallback(thisTimer.TasksExecuted, handleR, handleL, handleO, ...
    squeeze(RDATA(:, thisTimer.TasksExecuted, :)),...
    squeeze(LDATA(:, thisTimer.TasksExecuted, :)),...
    squeeze(ODATA(:, thisTimer.TasksExecuted, :)),...
    Order);
% Create an stopping function
t.StopFcn = @(~,thisEvent)fprintf("Animation ended.\n");
% The period of the function callback in seconds
t.Period = Ts;
% Number of times to perform the callback
t.TasksToExecute = N;
% Parameter related to queuing. Fixed rate means that the callback is
% executed as soon as it is added to the queue.
t.ExecutionMode = 'fixedRate';

% Start the timer
start(t);

%%

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

function flag = stopSearch(cnt, data, maxmin)
    % If data is less than percent of peak to peak value continue search
    if data(cnt) < maxmin
        flag = false;
        return
    end
    
    % Else if data is bigger, take it if it is a maximum in a local area of
    % given size
    size = 10;
    
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

function animateCallback(n, handleR, handleL, handleO, RDATA, LDATA, ODATA, Order)
    % Plot those data
    handleR(1).XData = RDATA(1, :);
    handleR(1).YData = RDATA(2, :);
    handleR(1).ZData = RDATA(3, :);
    
    handleR(2).XData = RDATA(1, Order);
    handleR(2).YData = RDATA(2, Order);
    handleR(2).ZData = RDATA(3, Order);
    
    handleL(1).XData = LDATA(1, :);
    handleL(1).YData = LDATA(2, :);
    handleL(1).ZData = LDATA(3, :);
    
    handleL(2).XData = LDATA(1, Order);
    handleL(2).YData = LDATA(2, Order);
    handleL(2).ZData = LDATA(3, Order);
    
    handleO.XData = ODATA(1, :);
    handleO.YData = ODATA(2, :);
    handleO.ZData = ODATA(3, :);
end