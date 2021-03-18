%TEST_ANIMATE_TOOLPOINTS is a script testing the ANIMATE_TOOLPOINTS
%function.
clear all; close all; clc;

%% Test function for hand type tool: For 2 time samples

% You need the orientation of the tool
toolOrientation = [0, pi/4];

% X position of the end effector at which tool should be attached
Xe = [0, 4];

% Y position of the end effector at which tool should be attached
Ye = [1, -5];

% Tool type
tool = struct("type", "hand", "length", 1);

% Caclulate tool mesh positions
[Xt, Yt] = Animate_ToolPoints(toolOrientation, Xe, Ye, tool);

% Extract the robot mesh at both position
Xr1 = [Xe(1) - cos(toolOrientation(1)), Xe(1)];
Yr1 = [Ye(1) - sin(toolOrientation(1)), Ye(1)];

Xr2 = [Xe(2) - cos(toolOrientation(2)), Xe(2)];
Yr2 = [Ye(2) - sin(toolOrientation(2)), Ye(2)];

% Extract the tool mesh at both positions
Xt1 = Xt(:, 1);
Yt1 = Yt(:, 1);

Xt2 = Xt(:, 2);
Yt2 = Yt(:, 2);

% Plot meshes
figure;
hold on;
plot(Xt1, Yt1, 'Color', [0 0 1], 'LineWidth', 2, 'DisplayName', 'HandToolConfig1');
plot(Xr1, Yr1, 'Color', [0 0.5 0.7], 'LineWidth', 2, 'DisplayName', 'RobotLastSegment1');
plot(Xt2, Yt2, 'Color', [1 0 0], 'LineWidth', 2, 'DisplayName', 'HandToolConfig2');
plot(Xr2, Yr2, 'Color', [0.7 0.5 0], 'LineWidth', 2, 'DisplayName', 'RobotLastSegment2');
grid;
legend;
axis equal;
xlabel('x-coordinate [m]');
ylabel('y-coordinate [m]');
title('Plotting a hand tool');

%% Test function for circle type: 2 samples


% You need the orientation of the tool
toolOrientation = [0, pi/4];

% X position of the end effector at which tool should be attached
Xe = [0, -3];

% Y position of the end effector at which tool should be attached
Ye = [1, +4];

% Tool type
tool = struct("type", "circle", "diameter", 1.5, "numPoints", 50);

% Caclulate tool mesh positions
[Xt, Yt] = Animate_ToolPoints(toolOrientation, Xe, Ye, tool);

% Extract the robot mesh at both position
Xr1 = [Xe(1) - cos(toolOrientation(1)), Xe(1)];
Yr1 = [Ye(1) - sin(toolOrientation(1)), Ye(1)];

Xr2 = [Xe(2) - cos(toolOrientation(2)), Xe(2)];
Yr2 = [Ye(2) - sin(toolOrientation(2)), Ye(2)];

% Extract the tool mesh at both positions
Xt1 = Xt(:, 1);
Yt1 = Yt(:, 1);

Xt2 = Xt(:, 2);
Yt2 = Yt(:, 2);

% Plot meshes
figure;
hold on;
plot(Xt1, Yt1, 'Color', [0 0 1], 'LineWidth', 2, 'DisplayName', 'CircleToolConfig1');
plot(Xr1, Yr1, 'Color', [0 0.5 0.7], 'LineWidth', 2, 'DisplayName', 'RobotLastSegment1');
plot(Xt2, Yt2, 'Color', [1 0 0], 'LineWidth', 2, 'DisplayName', 'CircleToolConfig1');
plot(Xr2, Yr2, 'Color', [0.7 0.5 0], 'LineWidth', 2, 'DisplayName', 'RobotLastSegment2');
grid;
legend;
axis equal;
xlabel('x-coordinate [m]');
ylabel('y-coordinate [m]');
title('Plotting a circle tool');
