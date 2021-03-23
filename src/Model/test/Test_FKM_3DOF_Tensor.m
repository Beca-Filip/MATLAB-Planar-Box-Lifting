%TEST_FKM_3DOF_TENSOR is a script that shows how to use the FKM_3DOF_TENSOR
%function.
clear all; close all; clc;

% Sampling rate
Ts = 0.01;

% Number of samples
N = 1000;

% Time vector
t = 0 : Ts : (N-1) * Ts;

% Define joint angle vectors
q = [sin(t); cos(t); sin(2*t)];

% Define robot dimensions
L = [0.5, 0.35, 0.2];

% Call function
T = FKM_3DOF_Tensor(q, L);

% Extract the profiles of x and y coordinates of end effector through time
Xee = squeeze(T(1, 4, 4, :));
Yee = squeeze(T(2, 4, 4, :));

% Extract the profiles of x and y coordinates of the 1st joint through time
Xj1 = squeeze(T(1, 4, 2, :));
Yj1 = squeeze(T(2, 4, 2, :));

% Extract the profiles of x and y coordinates of the 2nd joint through time
Xj2 = squeeze(T(1, 4, 3, :));
Yj2 = squeeze(T(2, 4, 3, :));

% Plot
figure;
subplot(2, 1, 1)
hold on;
plot(t, Xee, 'DisplayName', 'X_{EndEffector}');
plot(t, Xj1, 'DisplayName', 'X_{FirstJoint}');
plot(t, Xj2, 'DisplayName', 'X_{SecondJoint}');
xlabel('time [s]');
ylabel('x-coordinate [m]');
legend;
grid;
title('X coordinate of end effector through time');

subplot(2, 1, 2)
hold on;
plot(t, Yee, 'DisplayName', 'Y_{EndEffector}');
plot(t, Yj1, 'DisplayName', 'Y_{FirstJoint}');
plot(t, Yj2, 'DisplayName', 'Y_{SecondJoint}');
xlabel('time [s]');
ylabel('y-coordinate [m]');
legend;
grid;
title('Y coordinate of the end effector through time');