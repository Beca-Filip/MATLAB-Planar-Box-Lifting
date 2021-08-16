%% Flags
AppendixFlags.JointTrajectories = true;
AppendixFlags.JointTorques = true;
AppendixFlags.JointAccelerations = false;
AppendixFlags.JointJerks = false;
AppendixFlags.JointPowers = false;

AppendixFlags.TorquesSquared = false;
AppendixFlags.AccelerationsSquared = false;
AppendixFlags.JerksSquared = false;
AppendixFlags.PowersSquared = false;

AppendixFlags.VisualComparison = false;
AppendixFlags.NumericalComparison = true;

%% Get human trajectories and compate with optimal ones

% Human knots
q_knots_human = q(:, itpParam.KnotIndices);

% Get spline coeffs
polycoeffs_human = cell(1, NJ);
for ii = 1 : NJ
    polycoeffs_human{ii}  = splineInterpolation2(Tknots, q_knots_human(ii, :), ItpOrder, BndCnd);
end

% Get trajectories, velocities, accelerations, and jerks and separate them
q_tva = cell(1, NJ);
cell_q_human = cell(1, NJ);
cell_dq_human = cell(1, NJ);
cell_ddq_human = cell(1, NJ);
cell_dddq_human = cell(1, NJ);

for ii = 1 : NJ
    q_tva{ii} = splineCoefToTrajectory(Tknots, polycoeffs_human{ii}, Time, 3);
    cell_q_human{ii} = q_tva{ii}(1, :).'; % Transpose for later
    cell_dq_human{ii} = q_tva{ii}(2, :).'; % Transpose for later
    cell_ddq_human{ii} = q_tva{ii}(3, :).'; % Transpose for later 
    cell_dddq_human{ii} = q_tva{ii}(4, :).'; % Transpose for later    
end

% Merge trajectories velocities and accelerations into one matrix
q_human = [cell_q_human{:}].'; % Merge and transpose so rows correspond to single joint trajectories
dq_human = [cell_dq_human{:}].'; % Merge and transpose so rows correspond to single joint velocities
ddq_human = [cell_ddq_human{:}].'; % Merge and transpose so rows correspond to single joint accelerations
dddq_human = [cell_dddq_human{:}].'; % Merge and transpose so rows correspond to single joint jerks

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% Make 2d vector of ones for plotting
Ones = ones(1, 2);

% How many rows
figcols = 2;
figrows = 3;

% If flag is on do the plots
if AppendixFlags.JointTrajectories
    
figure;
sgtitle('Comparison of optimization-based and human joint trajectories');

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, q_star(curr,:), 'DisplayName', ['q_{' Joints{curr} '}-opt']);
        plot(Time, q_human(curr, :), 'DisplayName', ['q_{' Joints{curr} '}-hum']);
        plot(Time([1, end]), Ones*modelParam.JointLimits(1, curr), 'k--', 'DisplayName', ['q_{' Joints{curr} '}-lb']);
        plot(Time([1, end]), Ones*modelParam.JointLimits(2, curr), 'k--', 'DisplayName', ['q_{' Joints{curr} '}-ub']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angle [rad]']);
        title([Joints{curr} ' joint trajectory']);
        legend('Location', 'Best');
    end
end

% #IFFLAG end
end

%% Calculate Torques for Human Trajectories and Compare with Optimal Trajectories

% Get the external wrenches object
EW = getExternalWrenches(q,L,LiftParam);

% Calculate the dynamics for the optimal and human trajectories
[GAMMA_star, EN_star] = Dyn_6DOF(q_star, dq_star, ddq_star, modelParam, EW);
[GAMMA_human, EN_human] = Dyn_6DOF(q_human, dq_human, ddq_human, modelParam, EW);

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};

% Make 2d vector of ones for plotting
Ones = ones(1, 2);

% How many rows
figcols = 2;
figrows = 3;

% If flag is on do the plots
if AppendixFlags.JointTorques
    
figure;
sgtitle('Comparison of optimization-based and human joint torques');

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint torques
        plot(Time, GAMMA_star(curr,:), 'DisplayName', ['\Gamma_{' Joints{curr} '}-opt']);
        plot(Time, GAMMA_human(curr, :), 'DisplayName', ['\Gamma_{' Joints{curr} '}-hum']);
        plot(Time([1, end]), -Ones*modelParam.TorqueLimits(1, curr), 'k--', 'DisplayName', ['\Gamma_{' Joints{curr} '}-lb']);
        plot(Time([1, end]), Ones*modelParam.TorqueLimits(1, curr), 'k--', 'DisplayName', ['\Gamma_{' Joints{curr} '}-ub']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' torque [Nm]']);
        title([Joints{curr} ' torque profile']);
        legend('Location', 'Best');
    end
end

% IFFLAG end
end

%% Compare Human Joint Accelerations with Optimal Joint Accelerations

% If flag is on do the plots
if AppendixFlags.JointAccelerations
    
figure;
sgtitle('Comparison of optimization-based and human joint accelerations');

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint accelerations
        plot(Time, ddq_star(curr,:), 'DisplayName', ['ddq_{' Joints{curr} '}-opt']);
        plot(Time, ddq_human(curr, :), 'DisplayName', ['ddq_{' Joints{curr} '}-hum']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angular accel. [rad/s^2]']);
        title([Joints{curr} ' angular acceleration']);
        legend('Location', 'Best');
    end
end

% IFFLAG end
end

%% Compare Joint Jerks between Human and Optimization based trajectory

% If flag is on do the plots
if AppendixFlags.JointJerks
    
figure;
sgtitle('Comparison of optimization-based and human joint jerks');

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint jerks
        plot(Time, dddq_star(curr,:), 'DisplayName', ['dddq_{' Joints{curr} '}-opt']);
        plot(Time, dddq_human(curr, :), 'DisplayName', ['dddq_{' Joints{curr} '}-hum']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angular jerk [rad/s^3]']);
        title([Joints{curr} ' angular jerk']);
        legend('Location', 'Best');
    end
end

% IFFLAG end
end

%% Compare Joint Powers in Time

% Compute powers
P_star = GAMMA_star .* dq_star;
P_human = GAMMA_human .* dq_human;

% If flag is on do the plots
if AppendixFlags.JointPowers
    
figure;
sgtitle('Comparison of optimization-based and human joint powers');

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint jerks
        plot(Time, P_star(curr,:), 'DisplayName', ['P_{' Joints{curr} '}-opt']);
        plot(Time, P_human(curr, :), 'DisplayName', ['P_{' Joints{curr} '}-hum']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' power [W]']);
        title([Joints{curr} ' joint power']);
        legend('Location', 'Best');
    end
end

% IFFLAG end
end

%% Compare Sum of Squared Normalized Joint Torques of Human and Optimal Solution

% Sum the squares of normalised torque across joints
GAMMA_SN_star = sum((GAMMA_star ./ modelParam.TorqueLimits.').^2, 1) / size(GAMMA_star, 2) / modelParam.NJoints;
GAMMA_SN_human = sum((GAMMA_human ./ modelParam.TorqueLimits.').^2 , 1) / size(GAMMA_human, 2) / modelParam.NJoints;

% If flag is on do the plots
if AppendixFlags.TorquesSquared
    
% Plot
figure;
hold on;
plot(Time, GAMMA_SN_star, 'DisplayName', '\Gamma_{(N)}^{2}-opt');
plot(Time, GAMMA_SN_human, 'DisplayName', '\Gamma_{(N)}^{2}-hum');
xlabel('Time [s]');
ylabel('Normalised torque [None]');
legend('Location', 'Best');
title({'Comparison of optimisation based and'; 'human sum of normalised squared torques'});

% IFFLAG end
end
%% Compare Square Joint Accelerations

% Get the squared joint acceleration of initial trajectory
ddq_S_star = sum(ddq_star.^2, 1) / size(ddq_star, 2) / modelParam.NJoints;
% Get the squared joint acceleration of optimal trajectory
ddq_S_human = sum(ddq_human.^2, 1) / size(ddq_human, 2) / modelParam.NJoints;


% If flag is on do the plots
if AppendixFlags.AccelerationsSquared
    
% Plot
figure;
hold on;
plot(Time, ddq_S_star, 'DisplayName', 'ddq_{(N)}^{2}-opt');
plot(Time, ddq_S_human, 'DisplayName', 'ddq_{(N)}^{2}-hum');
xlabel('Time [s]');
ylabel('Accelerations [rad^2/s^4]');
legend('Location', 'Best');
title({'Comparison of optimisation based and'; 'human sum squared accelerations'});

end
%% Compare squared joint jerks between Human and Optimization based trajectories


% Get the squared joint acceleration of initial trajectory
dddq_S_star = sum(dddq_star.^2, 1) / size(dddq_star, 2) / modelParam.NJoints;
% Get the squared joint acceleration of optimal trajectory
dddq_S_human = sum(dddq_human.^2, 1) / size(dddq_human, 2) / modelParam.NJoints;


% If flag is on do the plots
if AppendixFlags.JerksSquared
    
% Plot
figure;
hold on;
plot(Time, dddq_S_star, 'DisplayName', 'dddq_{(N)}^{2}-opt');
plot(Time, dddq_S_human, 'DisplayName', 'dddq_{(N)}^{2}-hum');
xlabel('Time [s]');
ylabel('Jerks^2 [rad^2/s^6]');
legend('Location', 'Best');
title({'Comparison of optimisation based and'; 'human sum squared jerks'});

% IFFLAG end
end
%% Compare squared joint powers across time

% Squared and normalised joint powers
P_SN_star = sum((P_star ./ modelParam.TorqueLimits.').^2, 1) / size(P_star, 2) / modelParam.NJoints;
P_SN_human = sum((P_human ./ modelParam.TorqueLimits.').^2, 1) / size(P_human, 2) / modelParam.NJoints;


% If flag is on do the plots
if AppendixFlags.PowersSquared
    
% Plot
figure;
hold on;
plot(Time, P_SN_star, 'DisplayName', 'P_{(N)}^{2}-opt');
plot(Time, P_SN_human, 'DisplayName', 'P_{(N)}^{2}-hum');
xlabel('Time [s]');
ylabel({'(Norm. by Torque)'; 'Power^2 [rad^2/s^2]'});
legend('Location', 'Best');
title({'Comparison of optimisation based and'; 'human sum squared joint powers'});

% IFFLAG end
end

%% Compare values of approximations of cost functions in a bar graph

% Function names
Fnames = {'Torque', 'Acceleration', 'Jerk', 'Power'};

% Trajectories we are comparing
CNames = {'Optimal', 'Human'};

% Stack function values
J_star = [sum(GAMMA_SN_star), sum(ddq_S_star), sum(dddq_S_star), sum(P_SN_star)];
J_human = [sum(GAMMA_SN_human), sum(ddq_S_human), sum(dddq_S_human), sum(P_SN_human)];


% If flag is on do the plots
if AppendixFlags.VisualComparison
    
% Put those in a bar graph
barValues = [J_star; J_human];
barValues = barValues ./ max(barValues);
numbars = size(barValues, 2);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
rng(683);
for ii = 1 : size(barValues, 1)
    barChart(ii).FaceColor = 'flat';    % Let the facecolors be controlled by CData
%     barChart(ii).CData = repmat(linspace(0.2*ii, 0.4*ii, numbars)', 1, 3);     % Set CData
    barChart(ii).CData = repmat(double([mod(ii, 3) == 1, mod(ii, 3) == 2, mod(ii, 3) == 0]), numbars, 1);     % Set CData
    barChart(ii).DisplayName = CNames{ii};
end
xticks(barLocations);
xticklabels(Fnames);
xtickangle(30);
ylabel('Objective relative sizes');
title({'Visual comparison of multi-objective criteria between' ; 'a trajectory obtained from human measurements'; 'and a trajectory obtained from optimization'});
legend('Location', 'Best');

% IFFLAG end
end

%% Compare function values when pumped through cost function

% Optimization variables
x_human = q_knots_human.';
x_human = x_human(:).';

% Optimization function values
CF_star = full(costFunctionSet(x_star));
CF_human = full(costFunctionSet(x_human));


% If flag is on do the plots
if AppendixFlags.NumericalComparison
    
barValues = [CF_star; CF_human];
numbars = size(barValues, 2);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
for ii = 1 : size(barValues, 1)
    barChart(ii).FaceColor = 'flat';    % Let the facecolors be controlled by CData
%     barChart(ii).CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
    barChart(ii).CData = repmat(double([mod(ii, 3) == 1, mod(ii, 3) == 2, mod(ii, 3) == 0]), numbars, 1);     % Set CData
    barChart(ii).DisplayName = CNames{ii};
end
xticks(barLocations);
xticklabels(Fnames);
xtickangle(30);
ylabel('Objective Value');
title({'Numerical comparison of multi-objective criteria between' ; 'a trajectory obtained from human measurements'; 'and a trajectory obtained from optimization'});
legend('Location', 'Best');

% IFFLAG end
end

%% Show figures in generated order
lastFigNum = get(gcf, 'Number');
for ii = lastFigNum : -1 : 1
    figure(ii);
end