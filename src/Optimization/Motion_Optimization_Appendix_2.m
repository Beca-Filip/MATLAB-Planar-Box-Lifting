%% Flags
AppendixFlags.PlotCOP = true;
AppendixFlags.PlotNumericalHumanData = true;
AppendixFlags.PlotExtWrenches = true;

%% Get human trajectories without interpolation

% Get the velocity and acceleration
dq = diff(q, 1, 2) / Ts;
ddq = diff(q, 2, 2) / Ts^2;
dq = [dq, dq(:, end)];
ddq = [ddq, ddq(:, end-1:end)];

%% Get human trajectories and compate with optimal ones

% Human knots
q_knots_human = q(:, itpParam.KnotIndices);
x_human = q_knots_human.';
x_human = x_human(:).';

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

%% Plot CoP

% Get the external wrenches object
EW = getExternalWrenches(q, L, LiftParam);
EW_star = getExternalWrenches(q_star,L,LiftParam);
EW_human = getExternalWrenches(q_human,L,LiftParam);

% Get the COP
COP = COP_6DOF_Matrix(q, dq, ddq, modelParam, EW);
COP_star = COP_6DOF_Matrix(q_star, dq_star, ddq_star, modelParam, EW_star);
COP_human = COP_6DOF_Matrix(q_human, dq_human, ddq_human, modelParam, EW_human);

if AppendixFlags.PlotCOP
    % Plot the COP and the limits
    figure;
    hold on;
    if AppendixFlags.PlotNumericalHumanData; plot(Time, COP(1, :), 'DisplayName', 'COP'); end
    plot(Time, COP_star(1, :), 'DisplayName', 'COP-opt');
    plot(Time, COP_human(1, :), 'DisplayName', 'COP-human');
    plot(Time([1,end]), LiftParam.HeelPosition*ones(1,2), 'k--', 'DisplayName', 'COP-lb');
    plot(Time([1,end]), LiftParam.ToePosition*ones(1,2), 'k--', 'DisplayName', 'COP-ub');
    xlabel('Time [s]');
    ylabel('Displacement [m]');
    grid;
    legend;
    title('Comparison of human and optimization COP');
end

%% 

if AppendixFlags.PlotExtWrenches
    
    figure;
    
    subplot(3, 1, 1)
    hold on;
    if AppendixFlags.PlotNumericalHumanData; plot(Time, EW.EndEffectorWrenches(1, :), 'DisplayName', 'EW', 'LineWidth', 2); end
    plot(Time, EW_star.EndEffectorWrenches(1, :), 'DisplayName', 'EW-opt', 'LineWidth', 1.8);
    plot(Time, EW_human.EndEffectorWrenches(1, :), 'DisplayName', 'EW-human', 'LineStyle', '--', 'LineWidth', 1.6);
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid;
    legend;
    title({'Comparison of external force along the last segment'; 'for human and optimization trajectory'});
        
    subplot(3, 1, 2)
    hold on;
    if AppendixFlags.PlotNumericalHumanData; plot(Time, EW.EndEffectorWrenches(2, :), 'DisplayName', 'EW', 'LineWidth', 2); end
    plot(Time, EW_star.EndEffectorWrenches(2, :), 'DisplayName', 'EW-opt', 'LineWidth', 1.8);
    plot(Time, EW_human.EndEffectorWrenches(2, :), 'DisplayName', 'EW-human', 'LineStyle', '--', 'LineWidth', 1.6);
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid;
    legend;
    title({'Comparison of external force perpendicular to the last segment'; 'for human and optimization trajectory'});
    
    subplot(3, 1, 3)
    hold on;
    if AppendixFlags.PlotNumericalHumanData; plot(Time, EW.EndEffectorWrenches(6, :), 'DisplayName', 'EW', 'LineWidth', 2); end
    plot(Time, EW_star.EndEffectorWrenches(6, :), 'DisplayName', 'EW-opt', 'LineWidth', 1.8);
    plot(Time, EW_human.EndEffectorWrenches(6, :), 'DisplayName', 'EW-human', 'LineStyle', '--', 'LineWidth', 1.6);
    xlabel('Time [s]');
    ylabel('Moment [Nm]');
    grid;
    legend;
    title({'Comparison of external moment perpendicular to the plane of motion'; 'for human and optimization trajectory'});
end