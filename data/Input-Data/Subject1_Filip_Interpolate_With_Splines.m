clear all; close all; clc;

addpath('../../libs/splinePack/');
addpath('../../src/Model/');
addpath('../../src/Optimization/');

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

%% Define kinemato-dynamic parameters
q = lowpass_filter(q, 1/Ts, 2, 5);

% Get numerical velocity
dq = diff(q, 1, 2) / Ts;
ddq = diff(q, 2, 2) / Ts^2;
dq = [dq, dq(:, end)];
ddq = [ddq, ddq(:, end-1:end)];

% Get vectorized segment lengths
NJ = size(q, 1);
L = [];
for ii = 1 : NJ
    L = [L, modelParam.(['L', num2str(ii)])];
end

% External wrenches for measured motion
EW = getExternalWrenches(q, L, LiftParam);
% Get COP for measured motion
COP = COP_6DOF_Matrix(q,dq,ddq,modelParam,EW);

%% Spline interpolation parameters

% Create a structure containing interpolation parameters
% Number of knots (Must be an odd number because of the way we form the initial solution)
itpParam.NumControlPoints = 10;

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
    Indices = round(linspace(1, size(q, 2), Ncp));
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
    dq_h = [];
    ddq_h = [];
    for ii = 1 : NJ
        qdqddq_h = splineCoefToTrajectory(Tknots, coeffs(ii, :), Time, 2);
        q_h(ii, :) = qdqddq_h(1, :);
        dq_h(ii, :) = qdqddq_h(2, :);
        ddq_h(ii, :) = qdqddq_h(3, :);
    end
    
    % Get the vector of RMSE
    RMSEvec(jj, 1) = rmse(q, q_h);
    RMSEvec(jj, 2) = rmse(dq, dq_h);
    RMSEvec(jj, 3) = rmse(ddq, ddq_h);
    
    % Get external wrenches
    EW_h = getExternalWrenches(q_h, L, LiftParam);
    % Get COP
    COP_h = COP_6DOF_Matrix(q_h,dq_h,ddq_h,modelParam,EW);
    
    % Get the RMSE of COP's
    RMSEvec(jj, 4) = rmse(COP(1, :), COP_h(1, :));

end

figure;
subplot(4,1,1)
plot(NKP, RMSEvec(:, 1), 'DisplayName', 'Rmse');
xlabel('Num. knot points');
ylabel('RMSE [rad]');
title({'RMSE between interpolation and original trajectory'; 'for varying number of knot points'})
grid;
legend;
subplot(4,1,2)
plot(NKP, RMSEvec(:, 2), 'DisplayName', 'Rmse');
xlabel('Num. knot points');
ylabel('RMSE [rad/s]');
title({'RMSE between interpolation and original velocity'; 'for varying number of knot points'})
grid;
legend;
subplot(4,1,3)
plot(NKP, RMSEvec(:, 3), 'DisplayName', 'Rmse');
xlabel('Num. knot points');
ylabel('RMSE [rad/s^2]');
title({'RMSE between interpolation and original acceleration'; 'for varying number of knot points'})
grid;
legend;
subplot(4,1,4)
plot(NKP, RMSEvec(:, 4), 'DisplayName', 'Rmse');
xlabel('Num. knot points');
ylabel('RMSE [m]');
title({'RMSE between interpolation and original Center of Pressure position'; 'for varying number of knot points'})
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
qdqddq_h = [];

for ii = 1 : NJ
    qdqddq_h = splineCoefToTrajectory(Tknots, coeffs(ii, :), Time, 2);
    q_h(ii, :) = qdqddq_h(1, :);
    dq_h(ii, :) = qdqddq_h(2, :);
    ddq_h(ii, :) = qdqddq_h(3, :);
end

% Compare graphically
% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% How many rows
figcols = 2;
figrows = 3;

figure;
sgtitle({'Comparison of interpolation-based and human joint trajectories'; ['Num. itp. pts. = ' num2str(Ncp)]; ['RMSE = ' num2str(rmse(q, q_h),'%.4f') ' [rad]']});

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

figure;
sgtitle({'Comparison of interpolation-based and human joint velocities'; ['Num. itp. pts. = ' num2str(Ncp)]; ['RMSE = ' num2str(rmse(dq, dq_h),'%.4f') ' [rad/s]']});

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, dq(curr,:), 'DisplayName', ['dq_{' Joints{curr} '}-human']);
        plot(Time, dq_h(curr, :), 'DisplayName', ['dq_{' Joints{curr} '}-itp']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' velocity [rad/s]']);
        title([Joints{curr} ' joint velocity']);
        legend;
    end
end

figure;
sgtitle({'Comparison of interpolation-based and human joint accelerations'; ['Num. itp. pts. = ' num2str(Ncp)]; ['RMSE = ' num2str(rmse(ddq, ddq_h),'%.4f') ' [rad/s^2]']});

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, ddq(curr,:), 'DisplayName', ['ddq_{' Joints{curr} '}-human']);
        plot(Time, ddq_h(curr, :), 'DisplayName', ['q_{' Joints{curr} '}-itp']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' acceleration [rad/s^2]']);
        title([Joints{curr} ' joint acceleration']);
        legend;
    end
end

%% Look at center of pressure
% External wrenches
EW = getExternalWrenches(q, L, LiftParam);
EW_h = getExternalWrenches(q_h, L, LiftParam);
% Centers of pressure
COP = COP_6DOF_Matrix(q,dq,ddq,modelParam,EW);
COP_h = COP_6DOF_Matrix(q_h,dq_h,ddq_h,modelParam,EW);

figure;
hold on;
% Plot COP profile
plot(Time, COP(1,:), 'DisplayName', ['COP_{human}']);
plot(Time, COP_h(1, :), 'DisplayName', ['COP_{itp}']);
plot(Time([1, end]), ones(1,2)*LiftParam.HeelPosition, 'k--', 'DisplayName', 'Bounds');
plot(Time([1, end]), ones(1,2)*LiftParam.ToePosition, 'k--', 'HandleVisibility', 'Off');

% Labels
xlabel('Time [s]');
ylabel(['COP position [m]']);
title({'Comparison of interpolation-based and human Center of Pressure positions'; ['Num. itp. pts. = ' num2str(Ncp)]; ['RMSE = ' num2str(rmse(COP(1, :), COP_h(1, :)),'%.4f') ' [m]']});
legend;