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

%% Prepare plotting

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
view(-121.6899,-74.1249);

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