function Animate_2DOF(q, L, Ts, varargin)
%ANIMATE_2DOF animates a 2DOF planar robot, given the joint position
%vectors at successive samples, the robot segment lengths and the sampling
%rate.
%   ANIMATE_2DOF(q, L, Ts) takes in the matrix of joint angles q, the
%   size being (2 x Number of samples), the 2D vector of segment lengths L,
%   and the sampling rate Ts in secons (should be superior to 0.001s).
%   Creates a figure and animates the robot in a plane.
%
%   opts.tool = struct("type", "hand", "length", 1);
%   ANIMATE_2DOF(q, L, Ts, opts) plots an additional tool shape at the
%   end-effector location. The parameters of the tool should be given
%   according to the function ANIMATE_TOOLPOINTS.
%
%   opts.bgrPlot = @() plot(0.1*randn(1, 20), 0.1*randn(1, 20));
%   ANIMATE_2DOF(q, L, Ts, opts) takes in an additional function
%   bgrPlot, which plots someting in the background of the figure.
%
%   See also ANIMATE_TOOLPOINTS.
%   opts.generateLegend = true;
%   opts.legendParameters = {"Location", "SouthWest"}

% If options are passed
if nargin > 3
    % Initialize the opts variable
    opts = varargin{1};
end

% Exctract useful information
N = size(q, 2);
L1 = L(1);
L2 = L(2);
Ltool = min(L) * 0.25;

% Use the FMK_2DOF_Tensor function to find the the forward kinematics model
% of all joints frames
T = FKM_2DOF_Tensor(q, L);

% Extract the X and Y coordinates all segments across all times
X = squeeze(T(1, 4, :, :));
Y = squeeze(T(2, 4, :, :));

% Generate tool animation points
% Initialize toolplot to 0's 
Xtool = zeros(1, N);
Ytool = zeros(1, N);
% If the tool is indeed passed as an argument
if nargin > 3 && isfield(opts, "tool")
    % Assign the argument to a tool variable
    tool = opts.tool;        
    % Tool orientation at each timestep
    toolOrientation = sum(q);     
    % Call the tool mesh calculating function
    [Xtool, Ytool, Ltool] = Animate_ToolPoints(toolOrientation, X(end, :), Y(end, :), tool);
end

% Create the figure and initialize handles
figure;
hold all;

h_robot = plot(0, 0, 'LineWidth', 2, 'Color', 'k', 'DisplayName', 'RobotHand');     % Robot arm handle
h_joints = plot(0, 0, 'ko', 'MarkerSize', 10, 'DisplayName', 'RobotJoints');        % Robot joints handle
h_tool = plot(0, 0, 'LineWidth', 2, 'Color', 'k', 'HandleVisibility', 'Off');       % Robot tool handle

% Fix axis limits
x_low = -sum(L)-Ltool;
x_high = sum(L)+Ltool;
y_low = -sum(L)-Ltool;
y_high = sum(L)+Ltool;
xlim([x_low, x_high]);
ylim([y_low, y_high]);
title('2DOF Planar bot');
xlabel('X-axis [m]');
ylabel('Y-axis [m]');
grid;
pbaspect([1 1 1])

% Generate background plot
if nargin > 3 && isfield(opts, "bgrPlot")    
    % Load background plotting function handle into variable
    Animate_bgrPlot = opts.bgrPlot;
    
    % Call it
    Animate_bgrPlot();
end

% Generate legend (by default do not if not set by options
if nargin > 3 && isfield(opts, "generateLegend") && opts.generateLegend
    % If legend parameters are passed, pass them on
    if isfield(opts, "legendParameters")
        legend(opts.legendParameters{:});
    % Otherwise pass defaults
    else
        legend;
    end
end

% Create a timer to repeat the update of the plot handles
% Initialize it
t = timer;
% Create an starting function
% t.StartFcn = @(~,thisEvent)disp("Plotting...");
t.StartFcn = @timerBirthFunction;
% Create a callback function
t.TimerFcn = @(thisTimer, ~)updateHandles(h_robot, h_joints, h_tool, X(:, thisTimer.TasksExecuted), Y(:, thisTimer.TasksExecuted), Xtool(:, thisTimer.TasksExecuted), Ytool(:, thisTimer.TasksExecuted));
% Create an stopping function
% t.StopFcn = @(~,thisEvent)disp("Plotting ended.");
t.StopFcn = @timerKillFunction;
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

function updateHandles(h_robot, h_joints, h_tool, X, Y, Xtool, Ytool)
% Update robot arm handle as the piece-wise line segment whose points are
% the robots joint frame coordinates
h_robot.XData = X;
h_robot.YData = Y;

% Update the robot joints handle as the joint frame centers, exclude
% end-effector
h_joints.XData = X(1:end-1);
h_joints.YData = Y(1:end-1);

% Update the tool handle
h_tool.XData = Xtool;
h_tool.YData = Ytool;
end

function timerBirthFunction(thisTimer, thisEvent)
fprintf("Animation running...\n");
end
function timerKillFunction(thisTimer, thisEvent)
fprintf("Animation ended.\n");
delete(thisTimer);
end