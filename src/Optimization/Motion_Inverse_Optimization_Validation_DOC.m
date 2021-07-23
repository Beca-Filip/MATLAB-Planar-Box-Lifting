% Define some parameters for DOC
DefaultConstraintTolerance = 1e-3;
Ts = 0.01;
N = itpParam.KnotIndices(end);
Time = 0 : Ts : (N-1) * Ts;

% Redefine the optParam structure with the new weights
optParam2 = optParam;
optParam2.CostFunctionWeights = alpha_ioc';

% Get the linear constraint matrices
[A_star, b_star, Aeq_star, beq_star] = generateLinearConstraints(itpParam, optParam2, modelParam, LiftParam);

% Optimization options
op = optimoptions('fmincon',...   
                  'Algorithm', 'sqp',...
                  'ConstraintTolerance', optParam.DefaultConstraintTolerance, ...
                  'Display', 'Iter', ...
                  'MaxIter', 1e4, ...
                  'MaxFunctionEvaluations', 2e5, ...
                  'SpecifyObjectiveGradient', true, ...
                  'SpecifyConstraintGradient', true,...
                  'TolFun', 1e-3, ...
                  'UseParallel', 'Always' ...
                  );

load InitialPoint.mat

% % Re-do optimization
% [x_validation, f_validation, ef_validation, out_validation, lbd_validation, grad_validation, hess_validation] = ...
%         fmincon(...
%             @(x)costFunctionWrap(x, optParam), ...
%             x0, A_star, b_star, Aeq_star, beq_star, OptResults.lb, OptResults.ub, ...
%             @(x)constraintFunctionWrap(x, optParam), ...
%             op ...
%         );
% Re-do optimization
[x_validation, f_validation, ef_validation, out_validation, lbd_validation, grad_validation, hess_validation] = ...
        fmincon(...
            @(x)costFunctionWrap(x, optParam2), ...
            x_star, A_star, b_star, Aeq_star, beq_star, OptResults.lb, OptResults.ub, ...
            @(x)constraintFunctionWrap(x, optParam2), ...
            op ...
        );
    
    
% Reconstruct trajectory
[q_validation, dq_validation, ddq_validation] = unpackSplines(x_validation, modelParam, itpParam, Time);
[q_star, dq_star, ddq_star] = unpackSplines(x_star, modelParam, itpParam, Time);

% Get rmse function
rmse = @(a, b) sqrt(sum((a-b).^2 / numel(a), 'all'));

% Plot results
% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};
% Make 2d vector of ones for plotting
Ones = ones(1, 2);
% How many rows
figcols = 2;
figrows = 3;
% Create figure
figure;
sgtitle({'Comparison of retrieved weight optimization and human joint trajectories';...
        ['Orig. weig. : ' num2str(optParam.CostFunctionWeights, '%.2f ')];....
        ['Retr. weig. : ' num2str(alpha_ioc', '%.2f ')];...
        ['Total RMSE = ', num2str(rad2deg(rmse(q_star, q_validation)), '%.4f'), 'deg']});

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, q_star(curr,:), 'DisplayName', ['q_{' Joints{curr} '}-original']);
        plot(Time, q_validation(curr, :), 'DisplayName', ['q_{' Joints{curr} '}-retrieved']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angle [rad]']);
        title({[Joints{curr} ' joint trajectory']; ['RMSE = ', num2str(rad2deg(rmse(q_star(curr, :), q_validation(curr, :))), '%.4f'), 'rad']});
        legend;
    end
end