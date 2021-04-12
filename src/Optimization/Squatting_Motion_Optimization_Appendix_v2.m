%% Calculate Initial Trajectories and Compare with Optimal Trajectories
% Get spline interpolation coefs for each joint initial trajectory
polycoefs1_0_cf = splineInterpolation(itpParam.KnotValues, q1_knot_0, p, bndcnd);
polycoefs2_0_cf = splineInterpolation(itpParam.KnotValues, q2_knot_0, p, bndcnd);
polycoefs3_0_cf = splineInterpolation(itpParam.KnotValues, q3_knot_0, p, bndcnd);

% Get spline interpolation coefs for each joint optimal trajectory
polycoefs1_star_cf = splineInterpolation(itpParam.KnotValues, q1_knot_star, p, bndcnd);
polycoefs2_star_cf = splineInterpolation(itpParam.KnotValues, q2_knot_star, p, bndcnd);
polycoefs3_star_cf = splineInterpolation(itpParam.KnotValues, q3_knot_star, p, bndcnd);

% Redefine time parameters to be in line with the cost function resolution
t_cf = linspace(t0, tf, itpParam.ItpResolutionCost);

% Get spline initial trajectories from coeffs
q1_0_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs1_0_cf, t_cf, 3);
q2_0_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs2_0_cf, t_cf, 3);
q3_0_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs3_0_cf, t_cf, 3);

% Get spline optimal trajectories from coeffs
q1_star_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs1_star_cf, t_cf, 3);
q2_star_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs2_star_cf, t_cf, 3);
q3_star_cf = splineCoefToTrajectory(itpParam.KnotValues, polycoefs3_star_cf, t_cf, 3);

% Pack initial trajectories into a single matrix
dddq_0_cf = [q1_0_cf(4, :); q2_0_cf(4, :); q3_0_cf(4, :)];
ddq_0_cf = [q1_0_cf(3, :); q2_0_cf(3, :); q3_0_cf(3, :)];
dq_0_cf = [q1_0_cf(2, :); q2_0_cf(2, :); q3_0_cf(2, :)];
q_0_cf = [q1_0_cf(1, :); q2_0_cf(1, :); q3_0_cf(1, :)];

% Pack optimal trajectories into a single matrix
dddq_star_cf = [q1_star_cf(4, :); q2_star_cf(4, :); q3_star_cf(4, :)];
ddq_star_cf = [q1_star_cf(3, :); q2_star_cf(3, :); q3_star_cf(3, :)];
dq_star_cf = [q1_star_cf(2, :); q2_star_cf(2, :); q3_star_cf(2, :)];
q_star_cf = [q1_star_cf(1, :); q2_star_cf(1, :); q3_star_cf(1, :)];

% Plot them
two1 = ones(1, 2);  % Helper
figure;

subplot(3, 1, 1)
hold on;
plot(t_cf, q_0_cf(1, :), 'b--', 'DisplayName', 'Initial Traj');
% plot(t(itpParam.KnotIndices), q1_knot_0, 'bo', 'HandleVisibility', 'Off');
plot(t_cf, q_star_cf(1, :), 'r', 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
% plot(t(itpParam.KnotIndices), q1_knot_star, 'ro', 'HandleVisibility', 'Off');
plot(t_cf([1 end]), modelParam.JointLimits(1, 1)*two1, 'k--', 'DisplayName', 'Lower bound');
plot(t_cf([1 end]), modelParam.JointLimits(2, 1)*two1, 'k--', 'DisplayName', 'Upper bound');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint angle [rad]');
title({'Comparison of 1^{st} joint trajectories'});
grid;

subplot(3, 1, 2)
hold on;
plot(t_cf, q_0_cf(2, :), 'b--', 'DisplayName', 'Initial Traj');
% plot(t(itpParam.KnotIndices), q2_knot_0, 'bo', 'HandleVisibility', 'Off');
plot(t_cf, q_star_cf(2, :), 'r', 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
% plot(t(itpParam.KnotIndices), q2_knot_star, 'ro', 'HandleVisibility', 'Off');
plot(t_cf([1 end]), modelParam.JointLimits(1, 2)*two1, 'k--', 'DisplayName', 'Lower bound');
plot(t_cf([1 end]), modelParam.JointLimits(2, 2)*two1, 'k--', 'DisplayName', 'Upper bound');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint angle [rad]');
title({'Comparison of 2^{nd} joint trajectories'});
grid;

subplot(3, 1, 3)
hold on;
plot(t_cf, q_0_cf(3, :), 'b--', 'DisplayName', 'Initial Traj');
% plot(t(itpParam.KnotIndices), q3_knot_0, 'bo', 'HandleVisibility', 'Off');
plot(t_cf, q_star_cf(3, :), 'r', 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
% plot(t(itpParam.KnotIndices), q3_knot_star, 'ro', 'HandleVisibility', 'Off');
plot(t_cf([1 end]), modelParam.JointLimits(1, 3)*two1, 'k--', 'DisplayName', 'Lower bound');
plot(t_cf([1 end]), modelParam.JointLimits(2, 3)*two1, 'k--', 'DisplayName', 'Upper bound');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint angle [rad]');
title({'Comparison of 3^{rd} joint trajectories'});
grid;

%% Initialize Overall Cost Function Vector

% Overall cost function vectors for initial and optimal trajectory
JOverall_0 = [];
JOverall_star = [];


%% Calculate Torques for Initial Trajectories and Compare with Optimal Trajectories

% Flags
plotTorqueLimits = true;

% Get the zero external wrenches object
ZEW = zeroExternalWrenches3DOF(size(q_star_cf, 2));

% Calculate the dynamics for optimal trajectory
[GAMMA_star, EN_star] = Dyn_3DOF(q_star_cf, dq_star_cf, ddq_star_cf, ZEW, modelParam);

% Calculate the dynamics for initial trajectory
[GAMMA_0, EN_0] = Dyn_3DOF(q_0_cf, dq_0_cf, ddq_0_cf, ZEW, modelParam);

% Compare graphically
figure;

subplot(3, 1, 1)
hold on;
plot(t_cf, GAMMA_0(1, :), 'b--', 'DisplayName', 'Torques of Initial Traj');
plot(t_cf, GAMMA_star(1, :), 'LineWidth', 2, 'DisplayName', 'Torques of Optimal Traj');
if plotTorqueLimits
    plot(t_cf([1 end]), -modelParam.TorqueLimits(1, 1)*two1, 'k--', 'DisplayName', 'Lower bound');
    plot(t_cf([1 end]), modelParam.TorqueLimits(1, 1)*two1, 'k--', 'DisplayName', 'Upper bound');
end
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint torque [Nm]');
title({'Comparison of 1^{st} joint torques'});
grid;

subplot(3, 1, 2)
hold on;
plot(t_cf, GAMMA_0(2, :), 'b--', 'DisplayName', 'Torques of Initial Traj');
plot(t_cf, GAMMA_star(2, :), 'LineWidth', 2, 'DisplayName', 'Torques of Optimal Traj');
if plotTorqueLimits
    plot(t_cf([1 end]), -modelParam.TorqueLimits(1, 2)*two1, 'k--', 'DisplayName', 'Lower bound');
    plot(t_cf([1 end]), modelParam.TorqueLimits(1, 2)*two1, 'k--', 'DisplayName', 'Upper bound');
end
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint torque [Nm]');
title({'Comparison of 2^{nd} joint torques'});
grid;

subplot(3, 1, 3)
hold on;
plot(t_cf, GAMMA_0(3, :), 'b--', 'DisplayName', 'Torques of Initial Traj');
plot(t_cf, GAMMA_star(3, :), 'LineWidth', 2, 'DisplayName', 'Torques of Optimal Traj');
if plotTorqueLimits
    plot(t_cf([1 end]), -modelParam.TorqueLimits(1, 3)*two1, 'k--', 'DisplayName', 'Lower bound');
    plot(t_cf([1 end]), modelParam.TorqueLimits(1, 3)*two1, 'k--', 'DisplayName', 'Upper bound');
end
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint torque [Nm]');
title({'Comparison of 3^{rd} joint torques'});
grid;

%% Compare Square of Normalized Joint Torques Between Initial and Optimal Solution

% Squared normalized torque limits
squaredTorqueLimits = modelParam.TorqueLimits.^2;

% Calculate sum of squares of normalized torque of initial trajectory
GAMMA_normalised_0 = sum(GAMMA_0.^2 ./ (modelParam.TorqueLimits.^2)') / 3 / size(q_0_cf, 2);

% Calculate sum of squares of normalized torque of optimal trajectory
GAMMA_normalised_star = sum(GAMMA_star.^2 ./ (modelParam.TorqueLimits.^2)') / 3 / size(q_star_cf, 2);

% Compare graphically
figure;

hold on;
plot(t_cf, GAMMA_normalised_0(1, :), 'b--', 'DisplayName', 'Torques of Initial Traj');
plot(t_cf, GAMMA_normalised_star(1, :), 'LineWidth', 2, 'DisplayName', 'Torques of Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('normalised joint torque [no units]');
title({'Comparison of normalised squared joint torques'});
grid;

% Crude values of the torque cost function cost
JT_0 = sum(GAMMA_normalised_0);
JT_star = sum(GAMMA_normalised_star);

barValues = [JT_0 JT_star];
numbars = length(barValues);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'Initial Trajectory', 'Optimal Trajectory'});
ylabel('Normalised Squared Torque Value');
title('Comparing Normalised Squared Torque Values');

% Add crude values to the overall cost functions
JOverall_0 = [JOverall_0 JT_0];
JOverall_star = [JOverall_star JT_star];

%% Compare Initial Joint Accelerations with Optimal Joint Accelerations

figure;

subplot(3, 1, 1)
hold on;
plot(t_cf, ddq_0_cf(1, :), 'b--', 'DisplayName', 'Initial Traj');
plot(t_cf, ddq_star_cf(1, :), 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint acceleration [rad / s^2]');
title({'Comparison of 1^{st} joint accelerations'});
grid;

subplot(3, 1, 2)
hold on;
plot(t_cf, ddq_0_cf(2, :), 'b--', 'DisplayName', 'Initial Traj');
plot(t_cf, ddq_star_cf(2, :), 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint acceleration [rad / s^2]');
title({'Comparison of 2^{nd} joint accelerations'});
grid;

subplot(3, 1, 3)
hold on;
plot(t_cf, ddq_0_cf(3, :), 'b--', 'DisplayName', 'Initial Traj');
plot(t_cf, ddq_star_cf(3, :), 'LineWidth', 2, 'DisplayName', 'Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint acceleration [rad / s^2]');
title({'Comparison of 3^{rd} joint accelerations'});
grid;

%% Compare Square Joint Accelerations and Time

% Get the squared joint acceleration of initial trajectory
ddq_0_squared = sum(ddq_0_cf.^2) / itpParam.ItpResolutionCost / 3;

% Get the squared joint acceleration of optimal trajectory
ddq_star_squared = sum(ddq_star_cf.^2) / itpParam.ItpResolutionCost / 3;

% Compare graphically
figure;

hold on;
plot(t_cf, ddq_0_squared(1, :), 'b--', 'DisplayName', 'Acceleration^2 of Initial Traj');
plot(t_cf, ddq_star_squared(1, :), 'LineWidth', 2, 'DisplayName', 'Acceleration^2 of Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint acceleration squared [rad^2 / s^4]');
title({'Comparison of squared joint accelerations'});
grid;

% Crude values of the accelerations cost function cost
JA_0 = sum(ddq_0_squared);
JA_star = sum(ddq_star_squared);

barValues = [JA_0 JA_star];
numbars = length(barValues);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'Initial Trajectory', 'Optimal Trajectory'});
ylabel('Squared Joint Acceleration Value [rad^2 / s^4]');
title('Comparing Squared Joint Acceleration Values');

% Add crude values to the overall cost functions
JOverall_0 = [JOverall_0 JA_0];
JOverall_star = [JOverall_star JA_star];

%% Compare Square Joint Jerks in Time and in Value

% Get the squared joint jerks of initial trajectory
dddq_0_squared = sum(dddq_0_cf.^2) / itpParam.ItpResolutionCost / 3;

% Get the squared joint jerks of optimal trajectory
dddq_star_squared = sum(dddq_star_cf.^2) / itpParam.ItpResolutionCost / 3;

% Compare graphically
figure;

hold on;
plot(t_cf, dddq_0_squared(1, :), 'b--', 'DisplayName', 'Jerk^2 of Initial Traj');
plot(t_cf, dddq_star_squared(1, :), 'LineWidth', 2, 'DisplayName', 'Jerk^2 of Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint jerk squared [rad^2 / s^6]');
title({'Comparison of squared joint jerks'});
grid;

% Crude values of the jerks cost function
JJ_0 = sum(dddq_0_squared);
JJ_star = sum(dddq_star_squared);

barValues = [JJ_0 JJ_star];
numbars = length(barValues);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'Initial Trajectory', 'Optimal Trajectory'});
ylabel('Squared Joint Jerk Value [rad^2 / s^6]');
title('Comparing Squared Joint Jerk Values');

% Add crude values to the overall cost functions
% JOverall_0 = [JOverall_0 JJ_0];
% JOverall_star = [JOverall_star JJ_star];

%% Compare Square Joint Powers in Time and in Value

% Get the squared joint powers of initial trajectory
POW_0_squared = sum((dq_0_cf .* GAMMA_0).^2 ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% Get the squared joint powers of optimal trajectory
POW_star_squared = sum((dq_star_cf .* GAMMA_star).^2 ./ (modelParam.TorqueLimits.^2)') / itpParam.ItpResolutionCost / 3;

% Compare graphically
figure;

hold on;
plot(t_cf, POW_0_squared(1, :), 'b--', 'DisplayName', 'Power^2 of Initial Traj');
plot(t_cf, POW_star_squared(1, :), 'LineWidth', 2, 'DisplayName', 'Power^2 of Optimal Traj');
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('joint power squared [W^2]');
title({'Comparison of squared joint powers'});
grid;

% Crude values of the torque cost function cost
JP_0 = sum(POW_0_squared);
JP_star = sum(POW_star_squared);

barValues = [JP_0 JP_star];
numbars = length(barValues);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'Initial Trajectory', 'Optimal Trajectory'});
ylabel('Squared Joint Power Value [W^2]');
title('Comparing Squared Joint Power Values');

% Add crude values to the overall cost functions
JOverall_0 = [JOverall_0 JP_0];
JOverall_star = [JOverall_star JP_star];

%% Comparing Overall Cost Function

% Normalize the cost function
JOverall_0 = JOverall_0 ./ optParam.CostFunctionNormalisation;
JOverall_star = JOverall_star ./ optParam.CostFunctionNormalisation;

% Compute the weighed value of the overall cost function
JOverall_0 = sum(JOverall_0 .* optParam.CostFunctionWeights);
JOverall_star = sum(JOverall_star .* optParam.CostFunctionWeights);


% Compare them graphically
barValues = [JOverall_0 JOverall_star];
numbars = length(barValues);
barLocations = 1:numbars;
figure;
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'Initial Trajectory', 'Optimal Trajectory'});
ylabel('Overall Cost Function Value');
title('Overall Cost Function Value');
%% Plotting of COP

% Flags
plotCOPLimits = true;

% Compute COP position for optimal trajectory
ZEW = zeroExternalWrenches3DOF(size(q_star_cf, 2));
COP_star = COP_3DOF_Matrix(q_star_cf,dq_star_cf,ddq_star_cf,ZEW,modelParam);
XCOP_star = COP_star(1, :);

% Compute COP position for initial trajectory
COP_0 = COP_3DOF_Matrix(q_0_cf,dq_0_cf,ddq_0_cf,ZEW,modelParam);
XCOP_0 = COP_0(1, :);

% Get the limits
XCOP_high = modelParam.HeelPosition(1, 1);
XCOP_low = modelParam.ToePosition(1, 1);


% Compare graphically
figure;
hold on;
plot(t_cf, XCOP_0, 'b--', 'DisplayName', 'X_{COP} of Initial Traj');
plot(t_cf, XCOP_star, 'LineWidth', 2, 'DisplayName', 'X_{COP} of Optimal Traj');
if plotCOPLimits
    plot(t_cf([1 end]), XCOP_low*two1, 'k--', 'DisplayName', 'Lower bound');
    plot(t_cf([1 end]), XCOP_high*two1, 'k--', 'DisplayName', 'Upper bound');
end
legend('Location','NorthEast');
xlabel('time [s]');
ylabel('Center of pressure x position [m]');
title({'Comparison of X-Coordinate of COP'});
grid;

