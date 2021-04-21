%% Form the expected retrieval from IOC

% IOC variables but extracted from the DOC
alpha_star = optParam.CostFunctionWeights;
lambda_star = [Storage.Results.lbd_star.eqlin.', Storage.Results.lbd_star.eqnonlin.'];
mu_star = [Storage.Results.lbd_star.ineqlin.', Storage.Results.lbd_star.ineqnonlin.', ...
           Storage.Results.lbd_star.lower.', Storage.Results.lbd_star.upper.'];

vars_star = [alpha_star, lambda_star, mu_star].';

% Compare graphically with the retrieved variables
figure;

% Compare weights first
subplot(3, 2, 1)
barValues = [alpha_star; alpha_ioc'];
numbars = length(alpha_ioc);
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['\alpha_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'Original'; 'Retrieved'})
xticks(barLocations);
xticklabels(barNames);
xtickangle(0);
ylabel('Values of params');
title({'Investigating the';'cost function parametrization'});
legend;

% Compare equality multipliers second
subplot(3, 2, 2)
barValues = [lambda_star; lambda_ioc'];
numbars = length(lambda_ioc);
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['\lambda_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'Original'; 'Retrieved'})
xticks(barLocations);
xticklabels(barNames);
xtickangle(0);
ylabel('Values of eq mult');
title({'Investigating the equality';'lagrange multipliers'});
legend;

% Compare nonlinear inequality multipliers third
% Too many multipliers => put only 10 in xticks
subplot(3, 2, [3 4]);
actInd = find(C_star >= 0); % Find indices of active inequalities
Nnli = length(C_star);  % Num. of nonlinear inequalities
plotValues = [mu_star(1:Nnli); mu_ioc(1:Nnli)'];
numbars = Nnli;
xLocations = 1:numbars;
tickLocations = round(linspace(1, numbars, 10));
tickNames = {};
for ii = 1 : length(tickLocations)
    tickNames{ii} = ['\mu_{' num2str(tickLocations(ii)) '}'];
end
hold on;
plotChart = plot(xLocations, plotValues);
set(plotChart, {'DisplayName'}, {'Original'; 'Retrieved'})
set(plotChart, {'LineWidth'}, {2; 1.5});
set(plotChart, {'LineStyle'}, {'-'; '-.'});
plotActive = plot(xLocations(actInd), mu_star(xLocations(actInd)), 'o');
set(plotActive, {'DisplayName'}, {'Active'})
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the nonlinear';'inequality lagrange multipliers'});
legend;

% Plot lower bound multipliers
subplot(3, 2, 5);
actInd = find(-x_star + lb_doc >= 0);   % Find active bound inequalities
Nlb = length(lb_doc); % Number of lower bound inequalities
plotValues = [mu_star(Nnli+1:Nnli+Nlb); mu_ioc(Nnli+1:Nnli+Nlb)']; % Multipliers corresponding to lower bounds
numbars = Nlb;
xLocations = Nnli + 1 : Nnli + Nlb;
tickLocations = round(linspace(Nnli + 1, Nnli + Nlb, 10));
tickNames = {};
for ii = 1 : length(tickLocations)
    tickNames{ii} = ['\mu_{' num2str(tickLocations(ii)) '}'];
end
hold on;
plotChart = plot(xLocations, plotValues);
set(plotChart, {'DisplayName'}, {'Original'; 'Retrieved'})
set(plotChart, {'LineWidth'}, {2; 1.5});
set(plotChart, {'LineStyle'}, {'-'; '-.'});
plotActive = plot(xLocations(actInd), mu_star(xLocations(actInd)), 'o');
set(plotActive, {'DisplayName'}, {'Active'});
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the lower bound';'inequality lagrange multipliers'});
legend;

% Plot upper bound multipliers
subplot(3, 2, 6);
actInd = find(x_star - ub_doc >= 0);    % Find active bound inequalities
Nub = length(ub_doc); % Number of upper bound inequalities
plotValues = [mu_star(Nnli+Nlb+1:Nnli+Nlb+Nub); mu_ioc(Nnli+Nlb+1:Nnli+Nlb+Nub)']; % Multipliers corresponding to upper bounds
numbars = Nub;
xLocations = Nnli+Nlb+1:Nnli+Nlb+Nub;
tickLocations = round(linspace(Nnli+Nlb+1, Nnli+Nlb+Nub, 10));
tickNames = {};
for ii = 1 : length(tickLocations)
    tickNames{ii} = ['\mu_{' num2str(tickLocations(ii)) '}'];
end
hold on;
plotChart = plot(xLocations, plotValues);
set(plotChart, {'DisplayName'}, {'Original'; 'Retrieved'});
set(plotChart, {'LineWidth'}, {2; 1.5});
set(plotChart, {'LineStyle'}, {'-'; '-.'});
plotActive = plot(xLocations(actInd), mu_star(xLocations(actInd)), 'o');
set(plotActive, {'DisplayName'}, {'Active'});
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the upper bound';'inequality lagrange multipliers'});
legend;

%% Redo DOC with retrieved weights

% Re-assign weights
optParam.CostFunctionWeights = alpha_ioc';

% Prepare the optimization pipeline: optimization options
op = optimoptions('fmincon',...   
                  'Algorithm', 'sqp',...
                  'Display', 'Iter', ...
                  'MaxIter', 1e4, ...
                  'MaxFunctionEvaluations', 2e5, ...
                  'SpecifyObjectiveGradient', true, ...
                  'SpecifyConstraintGradient', true,...
                  'TolFun', 1e-3, ...
                  'UseParallel', 'Always' ...
                  );
              
% Initialize at optimal point
x0 = x_star;
              
% Run the optimization
[x_star2, f_star2, ef_star2, out_star2, lbd_star2, grad_star2, hess_star2] = ...
        fmincon(...
            @(x)costFunctionWrap(x, optParam), ...
            x0, A_star, b_star, Aeq_star', beq_star, lb_doc, ub_doc, ...
            @(x)constraintFunctionWrap(x), ...
            op ...
        );
    
% Plot the difference between the results
figure;

% Plot both results
subplot(3,1,1)
hold on;
plot(x_star, 'b', 'LineWidth', 1.2, 'DisplayName', 'Original');
plot(x_star2, 'r-o', 'DisplayName', 'Retrieved');
xlabel('k-th dimension');
ylabel('opt var val');
title({'Comparing the optimal solution that we get with'; 'DOC weights and IOC retrieved weights'});
legend;
grid;

% Plot their squared difference
x_diff = (x_star - x_star2).^2;
subplot(3,1,2)
hold on;
plot(x_diff, 'DisplayName', 'sq diff');
xlabel('k-th dimension');
ylabel('var val diff');
title({'Squared difference between the optimal solution that we get with'; 'DOC weights and IOC retrieved weights'});
legend;
grid;

% Plot the difference in function value
subplot(3,1,3)
hold on;
barValues = [Storage.Results.f_star f_star2];
numbars = length(barValues);
barLocations = 1:numbars;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels({'DOC Optimal Trajectory', 'IOC Optimal Trajectory'});
ylabel('Cost Function Value');
title({"Comparison of optimal solution's cost function values"; "between DOC and IOC optimal"});