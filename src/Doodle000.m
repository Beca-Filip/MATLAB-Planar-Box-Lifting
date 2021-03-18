clear all; close all; clc;

%%
syms x [60, 1]
syms f(x)

f(x) = 1/length(x) * transpose(x) * x;

df = jacobian(f, x);

grad = matlabFunction(df, 'Vars', x);

%%

figure;
hold on;

h_robot = plot([0], [0], 'LineWidth', 2, 'Color', 'k');   % Robot arm handle
h_joints = plot([0], [0], 'ko', 'MarkerSize', 10);        % Robot joints handle

%%
t = timer;
t.StartFcn = @(~,thisEvent)disp([thisEvent.Type ' executed '...
    datestr(thisEvent.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);
% t.TimerFcn = @(~,thisEvent)disp([thisEvent.Type ' executed '...
%      datestr(thisEvent.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);
t.TimerFcn = @(myBro, ~)fprintf("%d\n", myBro.TasksExecuted);
t.StopFcn = @(~,thisEvent)disp([thisEvent.Type ' executed '...
    datestr(thisEvent.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);
t.Period = 2;
t.TasksToExecute = 3;
t.ExecutionMode = 'fixedRate';
start(t)
%%
q = [sin(0:0.01:10*pi);cos(0:0.01:10*pi)];
% q = [2*pi/3; 0];
% L = [1, 1];
L = [0.5, 0.2];
Ts = 0.005;
% Ts = 10;
tool = struct("type", "circle", "diameter", min(L)*0.5, "numPoints", 100)
Animate_2DOF(q, L, Ts, tool);
%%
T = FKM_2DOF_Tensor(q, L)

%%
q = [sin(0:0.01:10*pi); cos(0:0.01:10*pi); sin(0:0.01:10*pi)];
L = [0.5, 0.3, 0.15];
Ts = 0.001;

Animate_3DOF(q, L, Ts);

%%

A = zeros(2, 2, 2);
A(:, :, 1) = 1;
A(:, :, 2) = 2;

x = [0; 1];

dot(A, x, 3)

%%
[Xt, Yt] = Animate_ToolPoints(2*pi/3*ones(1, 100), zeros(1, 100), zeros(1, 100), struct("type", "hand", "length", 1));

figure;
plot(Xt(:, 1), Yt(:, 1), 'k', 'LineWidth', 2);
axis equal
pbaspect([1 1 1])

%% Test Newton_Raphson
fun = @(x) exp(-x)-0.5;
grad = @(x) -exp(-x);
x0 = 5;
maxiter = 10;

x = Newton_Raphson(fun, grad, 5, maxiter);