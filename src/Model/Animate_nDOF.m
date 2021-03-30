function Animate_nDOF(q, L, Ts, varargin)
%ANIMATE_nDOF animates a n-DOF planar robot, given the joint position
%vectors at successive samples, the robot segment lengths and the sampling
%rate.
%   ANIMATE_3DOF(q, L, Ts) takes in the matrix of joint angles q, the
%   size being (n x Number of samples), the nD vector of segment lengths L,
%   and the sampling rate Ts in seconds (should be superior to 0.001s).
%   Creates a figure and animates the robot in a plane.
%
%   opts.tool = struct("type", "hand", "length", 1);
%   ANIMATE_nDOF(q, L, Ts, opts) plots an additional tool shape at the
%   end-effector location. The parameters of the tool should be given
%   according to the function ANIMATE_TOOLPOINTS.
%
%   opts.bgrPlot = @() plot(0.1*randn(1, 20), 0.1*randn(1, 20));
%   ANIMATE_nDOF(q, L, Ts, opts) takes in an additional function
%   bgrPlot, which plots someting in the background of the figure.
%
%   opts.handleInits = {@()plot(1, 0)};
%   opts.callback = @(ii, handle) handle.YData = (2*pi*(ii / 50)))
%   ANIMATE_nDOF(q, L, Ts, opts) takes in two additional functions, 
%   handleInits and callback. handleInits is meant to initialize plot
%   handles which will later be updated by the callback function.
%   In the example we'll have a point which will have coordinates
%   (1,sin(2*pi*(ii/50)) where ii is the number of the current frame.
%
%   See also ANIMATE_TOOLPOINTS.

%% Options initialization
% #Input check and initialization of the options structure: opts
if nargin > 3
    % Initialize the opts variable with the value of the additional
    % argument
    opts = varargin{1};
else
    % Create empty opts
    opts = [];
end

%% Robot Forward Kinematics and Tool-Plotting
% Exctract useful information / constants
n = size(q, 1); % Number of joints
N = size(q, 2); % Number of samples
Ltool = min(L) * 0.25;

% Use the FMK_nDOF_Tensor function to find the the forward kinematics model
% of all joints frames
T = FKM_nDOF_Tensor(q, L);

% Extract the X and Y coordinates all segments across all times
X = squeeze(T(1, 4, :, :));
Y = squeeze(T(2, 4, :, :));

% Generate tool animation points
% Initialize toolplot to 0's 
Xtool = zeros(1, N);
Ytool = zeros(1, N);

% #Input check
if nargin > 3 && isfield(opts, "tool")
    % Assign the argument to a tool variable
    tool = opts.tool;        
    % Tool orientation at each timestep
    toolOrientation = sum(q);     
    % Call the tool mesh calculating function
    [Xtool, Ytool, Ltool] = Animate_ToolPoints(toolOrientation, X(end, :), Y(end, :), tool);
end

%% Figure creation and handle initialization
% #Storing Options: Create the figure, store its handle inside options
opts.Figure = figure;
hold all;

% Initialize handles for robot, joints and tool, and store them inside a
% single graphical objects array
h_robot = plot(0, 0, 'LineWidth', 2, 'Color', 'k', 'DisplayName', 'RobotHand');     % Robot arm handle
h_joints = plot(0, 0, 'ko', 'MarkerSize', 10, 'DisplayName', 'RobotJoints');        % Robot joints handle
h_tool = plot(0, 0, 'LineWidth', 2, 'Color', 'k', 'HandleVisibility', 'Off');       % Robot tool handle

h_all = [h_robot; h_joints; h_tool];

% #Input check and initialization of the handles given by opts
if nargin > 3 && isfield(opts, "handleInits")
    
    % #Storing Options: Initialize an array of graphical objects object
    opts.numHandles = numel(opts.handleInits);
    opts.handles = gobjects(1, opts.numHandles);
    
    % Storing Options: For each handle initialization
    for ii = 1 : opts.numHandles
        % Call the function to get handle initialised
        opts.handles(ii) = opts.handleInits{ii}();
    end
    disp(opts.handles);
end

%% Figure Axes, Legends, Aspect, Title, Plot Background
% Fix axis limits
x_low = -sum(L)-Ltool;
x_high = sum(L)+Ltool;
y_low = -sum(L)-Ltool;
y_high = sum(L)+Ltool;
xlim([x_low, x_high]);
ylim([y_low, y_high]);
% Titles and labels
title('3DOF Planar bot');
xlabel('X-axis [m]');
ylabel('Y-axis [m]');
grid;
pbaspect([1 1 1])

% #Input check and generation of a background plot
if nargin > 3 && isfield(opts, "bgrPlot")    
    % Load background plotting function handle into variable
    Animate_bgrPlot = opts.bgrPlot;
    
    % Call it
    Animate_bgrPlot();
end

% #Input check and generation of a legend
if nargin > 3 && isfield(opts, "generateLegend") && opts.generateLegend
    % If legend parameters are passed, pass them on
    if isfield(opts, "legendParameters")
        legend(opts.legendParameters{:});
    % Otherwise pass defaults
    else
        legend;
    end
end

%% Timer creation, Timer Data, Timer Start, End and Callback functions.

% Create a timer to repeat the update of the plot handles
% Initialize it
t = timer;

% #Input Check and storage of movie frames in timer field UserData
if nargin > 3 && isfield(opts, "saveas")
    % Preallocate a struct array variable within UserData in which to store 
    % frames
    for ii = N : -1 : 1
        t.UserData.savedFrames(ii).cdata = [];
        t.UserData.savedFrames(ii).colormap = [];
    end
    % #Storing Options: Save framerate in options structure
    opts.FrameRate = 1/Ts;
end

% Create an starting function
t.StartFcn = @timerBirthFunction;
% #Storing Options End, Use Options: Create a callback function
t.TimerFcn = @(thisTimer, ~)updateHandles(thisTimer, h_all, X, Y, Xtool, Ytool, opts);
% #Storing Options End, Use Options: Create a stopping function
t.StopFcn = @(thisTimer, thisEvent) timerKillFunction(thisTimer, thisEvent, opts);
% The period of the function callback in seconds
t.Period = Ts;
% Number of times to perform the callback
t.TasksToExecute = N;
% Parameter related to queuing. Fixed rate means that the callback is
% executed as soon as it is added to the queue.
t.ExecutionMode = 'fixedRate';

% Starting the timer
start(t);

end

function updateHandles(thisTimer, h_all, X, Y, Xtool, Ytool, opts)
    %UPDATEHANDLES gets the timer, the main handles, the main data and the
    %options. It updates all the main handles with the main data, following
    %predefined rules. It updates optional data trough callback stored in 
    %options. It also stores movie frames within the timer structure so
    %that they may be carried over out of this particular function.
    
    % Get the current iteration
    ii = thisTimer.TasksExecuted;
    
    %% Update Main Handles with Main Data
    % Update robot arm handle as the piece-wise line segment whose points are
    % the robots joint frame coordinates
    h_all(1).XData = X(:, ii);
    h_all(1).YData = Y(:, ii);

    % Update the robot joints handle as the joint frame centers, exclude
    % end-effector
    h_all(2).XData = X(1:end-1, ii);
    h_all(2).YData = Y(1:end-1, ii);

    % Update the tool handle
    h_all(3).XData = Xtool(:, ii);
    h_all(3).YData = Ytool(:, ii);

    %% Update Optional Data
    % #Input check and the update of all optional handles using the
    % optional callbacks
    if isfield(opts, "callback") && isfield(opts, "handles")
        % Call the callback function with desired handles and iteration number
        opts.callback(ii, opts.handles);
    end

    %% Frame Storage
    % #Input check and current frame storage
    if isfield(opts, "saveas")
        % Get current frame
        thisTimer.UserData.savedFrames(ii) = getframe(opts.Figure);
    end
end

function timerBirthFunction(thisTimer, thisEvent)
    %TIMERBIRTHFUNCTION prints a message to the user to anounce that the
    %animation is running.
    % Print message to user
    fprintf("Animation running...\n");
end
function timerKillFunction(thisTimer, thisEvent, opts)
    %TIMERKILLFUNCTION prints a message to the user to anounce that the
    %animation has ended. Optionally saves stored movie frames.
    
    % Print message to user
    fprintf("Animation ended.\n");

    %% Storing Movie Frames
    % #Input Check and storage of the movie
    if isfield(opts, "saveas")
        % Print message to user
        fprintf("Saving animation...\n");

        % Create a video writer object
        vw = VideoWriter(opts.saveas.name, 'Archival');
        
        % Set framerate parameter
        vw.FrameRate = opts.FrameRate;
        
        % Get the number of frames
        FrameCount = length(thisTimer.UserData.savedFrames);
        
        % Open the video for writing
        vw.open();
        
        % Write each frame
        for ii = 1 : FrameCount
            vw.writeVideo(thisTimer.UserData.savedFrames(ii));
        end
        
        % Close the video
        vw.close();
        
        % Print message to user
        fprintf(strcat("Animation saved as : ", opts.saveas.name, ".\n"));
    end
    
    % Delete the timer
    delete(thisTimer);
end