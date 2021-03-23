%TEST_FKM_2DOF_TENSOR is a script that shows how to use the FKM_2DOF_TENSOR
%function.
clear all; close all; clc;

% Sampling rate
Ts = 0.01;

% Number of samples
N = 1000;

% Time vector
t = 0 : Ts : (N-1) * Ts;

% Define joint angle vectors
q = [sin(t); cos(t)];

% Define robot dimensions
L = [0.5, 0.3];

% Call function
T = FKM_2DOF_Tensor(q, L);

% Extract the profiles of x and y coordinates of end effector through time
Xee = squeeze(T(1, 4, 3, :));
Yee = squeeze(T(2, 4, 3, :));

% Extract the profiles of x and y coordinates of the 1st joint through time
Xj1 = squeeze(T(1, 4, 2, :));
Yj1 = squeeze(T(2, 4, 2, :));

% Plot
figure;
subplot(2, 1, 1)
hold on;
plot(t, Xee, 'DisplayName', 'X_{EndEffector}');
plot(t, Xj1, 'DisplayName', 'X_{FirstJoint}');
xlabel('time [s]');
ylabel('x-coordinate [m]');
legend;
grid;
title('X coordinate of end effector through time');

subplot(2, 1, 2)
hold on;
plot(t, Yee, 'DisplayName', 'Y_{EndEffector}');
plot(t, Yj1, 'DisplayName', 'Y_{FirstJoint}');
xlabel('time [s]');
ylabel('y-coordinate [m]');
legend;
grid;
title('Y coordinate of the end effector through time');