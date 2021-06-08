%% Flags
AppendixFlags.PlotCOP = true;

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
EW_star = getExternalWrenches(q_star,L,LiftParam);
EW_human = getExternalWrenches(q_human,L,LiftParam);

% Get the COP
COP_star = COP_6DOF_Matrix(q_star, dq_star, ddq_star, modelParam, EW_star);
COP_human = COP_6DOF_Matrix(q_human, dq_human, ddq_human, modelParam, EW_human);

if AppendixFlags.PlotCOP
    % Plot the COP and the limits
    figure;
    hold on;
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