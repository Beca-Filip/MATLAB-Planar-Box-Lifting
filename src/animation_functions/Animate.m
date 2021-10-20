function Animate(anim_fun, N, Ts, varargin)
%ANIMATE uses timer callbacks with fixed period to call the user-provided
%animation function a given number of times, providing at each call the 
%index of the executed callback.
%   
%   ANIMATE(anim_fun, N, Ts) calls animation function N times with period
%   Ts, providing at each time the index of the executed callback. A call
%   to the animation function should then look like: anim_fun(ii).
%   
%   ANIMATE(anim_fun, N, Ts, options) allows the possibility of providing
%   optional arguments such as:
%       - save_path : where the function should save the animation along 
%       with the name of the file (e.g. '../../folder1/filename' or
%       '../../folder1/filename.avi').

%% Options initialization
% #Input check and initialization of the options structure: options
if nargin > 3
    % Initialize the options variable with the value of the additional
    % argument
    options = varargin{1};
else
    % Create empty options
    options = [];
end

%% Timer creation, Timer Data, Timer Start, End and Callback functions.

% Create a timer to repeat the update of the plot handles
% Initialize it
t = timer;

% Create an starting function
t.StartFcn = @timerBirthFunction;
% #Storing Options End, Use Options: Create a callback function
t.TimerFcn = @(thisTimer, thisEvent) timerCallbackFunction(thisTimer, thisEvent, anim_fun, options);
% #Storing Options End, Use Options: Create a stopping function
t.StopFcn = @(thisTimer, thisEvent) timerKillFunction(thisTimer, options);
% The period of the function callback in seconds
t.Period = Ts;
% Number of times to perform the callback
t.TasksToExecute = N;
% Parameter related to queuing. Fixed rate means that the callback is
% executed as soon as it is added to the queue.
t.ExecutionMode = 'fixedRate';

% #Input Check and storage of movie frames in timer field UserData
if nargin > 3 && isfield(options, "save_path")
    % Preallocate a struct array variable within UserData in which to store 
    % frames
    for ii = N : -1 : 1
        t.UserData.savedFrames(ii).cdata = [];
        t.UserData.savedFrames(ii).colormap = [];
    end
    % #Storing Options: Save framerate in options structure
    t.UserData.FrameRate = 1/Ts;
end

% Starting the timer
start(t);

end

function timerCallbackFunction(thisTimer, thisEvent, anim_fun, options)
    %TIMERCALLBACKFUNCTION performs the callback duties.
    
    % Get the current index
    ii = thisTimer.TasksExecuted;
    
    % Call the user-provided animation function
    anim_fun(ii);
    
    % If the options demand it
    if isfield(options, "save_path")
        thisTimer.UserData.savedFrames(ii) = getframe(gcf);
    end
end

function timerBirthFunction(thisTimer, thisEvent)
    %TIMERBIRTHFUNCTION prints a message to the user to anounce that the
    %animation is running.
    % Print message to user
    fprintf("Animation running...\n");
end

function timerKillFunction(thisTimer, options)
    %TIMERKILLFUNCTION prints a message to the user to anounce that the
    %animation has ended. Optionally saves stored movie frames.
    
    % Print message to user
    fprintf("Animation ended.\n");
    
    % Storing Movie Frames
    % #Input Check and storage of the movie
    if isfield(options, "save_path")
        
        % Print message to user
        fprintf("Saving animation...\n");

        % Create a video writer object
        vw = VideoWriter(strcat(options.save_path, '.avi'));
        
        % Set framerate parameter
        vw.FrameRate = thisTimer.UserData.FrameRate;
        
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
        fprintf(strcat("Animation saved as : ", options.save_path, ".\n"));
    end
    
    % Delete the timer
    delete(thisTimer);
end