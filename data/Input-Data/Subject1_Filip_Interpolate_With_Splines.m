clear all; close all; clc;

addpath('../../libs/splinePack/');

load Subject1_Filip_Segmented.mat

%% Define time related parameters
% Number of points
N = size(q, 2);

% Sampling rate
Ts = 0.01;

% Start time
t0 = 0;

% Final time
tf = (N-1)*Ts;

% Time vector
Time = linspace(t0, tf, N);

%% Spline interpolation parameters

% Create a structure containing interpolation parameters
% Number of knots (Must be an odd number because of the way we form the initial solution)
itpParam.NumControlPoints = 17;

% Spline Knot Indices
itpParam.KnotIndices = floor(linspace(1, N, itpParam.NumControlPoints));

% Spline Knot Values - abscissa - 1st joint
itpParam.KnotValues = Time(itpParam.KnotIndices);

% Order of interpolation
itpParam.InterpolationOrder = 5;

% Velocity and acceleration boundary conditions
itpParam.BoundaryConditions = [
              1, Time(1), 0;       % Zero velocity at first time
              2, Time(1), 0;       % Zero acceleration at first time
              1, Time(end), 0;     % Zero velocity at the last time
              2, Time(end), 0;     % Zero acceleration at the last time
];

%% Vary the parameters look at rmse
rmse = @(a, b) sqrt(sum((a-b).^2, 'all') / numel(a));

% Number of knot points
NKP = 2 : 60;

for jj = 1 : length(NKP)
    
    % Number of knots
    Ncp = NKP(jj);
    % Indices within original vectors
    Indices = floor(linspace(1, size(q, 2), Ncp));
    % Time at which the knots occur
    Tknots = Time(Indices);
    % Order of interpolation
    ItpOrder = 5;
    % Boundary Conditions
    BndCnd = itpParam.BoundaryConditions;

    % Number of joints
    NJ = 6;
    
    % Extract knots of individual joints
    q_knots = q(:, Indices);

    % Get coefficients
    coeffs = [];
    for ii = 1 : NJ
        coeffs(ii, :) = splineInterpolation2(Tknots, q_knots(ii, :), ItpOrder, BndCnd);
    end

    % Get trajectories
    q_h = [];
    for ii = 1 : NJ
        q_h(ii, :) = splineCoefToTrajectory(Tknots, coeffs(ii, :), Time, 0);
    end
    
    % Get the vector of RMSE
    RMSEvec(jj) = rmse(q, q_h);
end

figure;
plot(NKP, RMSEvec, 'DisplayName', 'Rmse');
xlabel('Num. knot points');
ylabel('RMSE [rad]');
title({'RMSE between interpolation and original'; 'for varying number of knot points'})
grid;
legend;
%%
% Define useful constant names
Ncp = itpParam.NumControlPoints;
Tknots =  itpParam.KnotValues;
ItpOrder = itpParam.InterpolationOrder;
BndCnd = itpParam.BoundaryConditions;

% Number of joints
NJ = 6;

% Extract knots of individual joints
q_knots = q(:, itpParam.KnotIndices);

% Get coefficients
coeffs = [];
for ii = 1 : NJ
    coeffs(ii, :) = splineInterpolation2(Tknots, q_knots(ii, :), ItpOrder, BndCnd);
end

% Get trajectories
q_h = [];
for ii = 1 : NJ
    q_h(ii, :) = splineCoefToTrajectory(Tknots, coeffs(ii, :), Time, 0);
end

% Compare graphically
% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% How many rows
figcols = 2;
figrows = 3;

figure;
sgtitle({'Comparison of optimization-based and human joint trajectories'; ['Num. itp. pts. = ' num2str(Ncp)]; ['RMSE = ' num2str(rmse(q, q_h),'%.4f') ' [rad]']});

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, q(curr,:), 'DisplayName', ['q_{' Joints{curr} '}-human']);
        plot(Time, q_h(curr, :), 'DisplayName', ['q_{' Joints{curr} '}-itp']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angle [rad]']);
        title([Joints{curr} ' joint trajectory']);
        legend;
    end
end