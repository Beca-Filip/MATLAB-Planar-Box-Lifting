function [J] = allSamplesCostFunction(x,timeParam,modelParam)
%ALLSAMPLESCOSTFUNCTION implements the desired cost function of a 
%3DOF model of a human performing a squat using all samples of a trajectory.

% Extract useful constraints
n = timeParam.NumSamples;
Ts = timeParam.SamplingPeriod;

% Unpack optimization variable
q1 = x(1:n);
q2 = x(n+1:2*n);
q3 = x(2*n+1:3*n);

% Initialize cost function vector
J = [];

%% Intermediate Computations: Get joint trajectories, velocities and accelerations

% Trajectories
q_full = [q1; q2; q3];

% Velocities
dq_full = diff(q_full, 1, 2) / Ts;    % 1st order difference along second dimension

% Accelerations
ddq_full = diff(dq_full, 1, 2) / Ts;  % 1st order difference along second dimension

% Jerks
dddq = diff(ddq_full, 1, 2) / Ts;    % 1st order difference along second dimension

% Pick only the trajectory and velocity samples that will be used to
% calculate the cost function
q = q_full(:, 1:end-2);
dq = dq_full(:, 1:end-1);
ddq = ddq_full(:, 1:end);

% Get the number of samples that is going to take part in the calculation
% of the cost function
N = size(q, 2);

%% Intermediate Computations: Joint Torques

% Generate zero external wrenches structure as required by Dyn_3DOF
ZEW = zeroExternalWrenches3DOF(N);

% Get wrenches from Dyn_3DOF
[GAMMA, ~] = Dyn_3DOF(q, dq, ddq, ZEW, modelParam);

% Get Joint Torque cost function
JT = sum(sum(GAMMA.^2, 2) ./ (modelParam.TorqueLimits.^2)') / N / 3;

% #Add to cost function vector
J = [J JT];

%% Intermediate Computations: Joint Acceleration

% Get Joint Acceleration cost function
JA = sum(sum(ddq.^2)) / N / 3;

% #Add to cost function vector
J = [J JA];

%% Intermediate Computations: Joint Jerk

% Get Joint Jerk cost function
JJ = sum(sum(dddq.^2)) / N / 3;

% #Add to cost function vector
% J = [J JJ];

%% Intermediate Computations: Joint Power

% Get Joint Power cost function
JP = sum(sum((dq .* GAMMA).^2, 2) ./ (modelParam.TorqueLimits.^2)') / N / 3;

% #Add to cost function vector
J = [J JP];

%% Final Computation:

% Cost Function
% J = J;
end

