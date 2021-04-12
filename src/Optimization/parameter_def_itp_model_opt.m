%% Define time related parameters
% Number of points
N = size(Trial(simParam.TrialNumber).q, 2);

% Sampling rate
Ts = 0.01;

% Start time
t0 = 0;

% Final time
tf = (N-1)*Ts;

% Time vector
t = linspace(t0, tf, N);


%% Spline interpolation parameters

% Create a structure containing interpolation parameters
% Number of knots (Must be an odd number because of the way we form the initial solution)
itpParam.NumControlPoints = 15;

% Spline Knot Indices
itpParam.KnotIndices = floor(linspace(1, N, itpParam.NumControlPoints));

% Spline Knot Values - abscissa - 1st joint
itpParam.KnotValues = t(itpParam.KnotIndices);

% Order of interpolation
itpParam.InterpolationOrder = 5;

% Velocity and acceleration boundary conditions
itpParam.BoundaryConditions = [
              1, t(1), 0;       % Zero velocity at first time
              2, t(1), 0;       % Zero acceleration at first time
              1, t(end), 0;     % Zero velocity at the last time
              2, t(end), 0;     % Zero acceleration at the last time
];

% How many points to interpolate with in optimization constraints
itpParam.ItpResolutionConstraints = 50;

% How many points to interpolate with in optimization cost calculation
itpParam.ItpResolutionCost = 50;

%% Define other input parameters to the model

% Initial squatting position (upright)
modelParam.InitialAngles = Trial(simParam.TrialNumber).q(:, 1);

% Final squatting position (upright)
modelParam.FinalAngles = Trial(simParam.TrialNumber).q(:, end);

% Crunch time in percentage of total time
modelParam.CrunchTimePercentage = Trial(simParam.TrialNumber).CrunchTimePercentage;

% Final squatting neck position percentage (add 3% margin)
modelParam.CrunchNeckHeightPercentage = Trial(simParam.TrialNumber).CrunchNeckHeightPercentage + 0.03;

% Final squatting hip position percentage (add 3% margin)
modelParam.CrunchHipHeightPercentage = Trial(simParam.TrialNumber).CrunchHipHeightPercentage + 0.03;

% Human starting, crunching and ending position of squat defined
q0 = Trial(simParam.TrialNumber).q(:, 1);
qc = Trial(simParam.TrialNumber).q(:, floor(Trial(simParam.TrialNumber).CrunchTimePercentage * length(Trial(simParam.TrialNumber).q)));
qf = Trial(simParam.TrialNumber).q(:, end);

%% Constraint tolerances

% Get default tolerance
DefaultConstraintTolerance = optimoptions('fmincon').ConstraintTolerance;

% Initial condition tolerances
TolInitialConditions = 1e-3;
optParam.MulInitialConditions = DefaultConstraintTolerance / TolInitialConditions;

% Final condition tolerances
TolFinalConditions = 1e-3;
optParam.MulFinalConditions = DefaultConstraintTolerance / TolFinalConditions;

% Crunch condition tolerances
TolCrunchConditions = 1e-3;
optParam.MulCrunchConditions = DefaultConstraintTolerance / TolCrunchConditions;

% Center of Pressure within bounds constraints
TolCOPConditions = 1e-3;  % in meters
optParam.MulCOPConditions = DefaultConstraintTolerance / TolCOPConditions;

% Torque limits constriants
TolTorqueLimits = 1e-3;
optParam.MulTorqueLimits = DefaultConstraintTolerance / TolTorqueLimits;