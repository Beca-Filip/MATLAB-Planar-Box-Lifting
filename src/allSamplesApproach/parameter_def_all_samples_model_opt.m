%PARAMETER_DEF_ALL_SAMPLES_MODEL_OPT is a script to be used withing other scripts
%(all samples Squattiong_Motion_Optimization variants) it generates the parameters
%related to the optimization contained in structures timeParam, modelParam
%and optParam.
%% Define time related parameters
% Number of samples
timeParam.NumSamples = size(Trial(simParam.TrialNumber).q, 2);

% Sampling rate
timeParam.SamplingPeriod = 0.01;

% Start time
timeParam.InitialTime = 0;

% Final time
timeParam.FinalTime = (timeParam.NumSamples-1)*timeParam.SamplingPeriod;

% Time vector
timeParam.TimeVectors = linspace(timeParam.InitialTime, timeParam.FinalTime, timeParam.NumSamples);

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