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
% Too many multipliers => take 10 in all
subplot(3, 2, [3 4]);
Nnli = length(C_star);
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
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the nonlinear';'inequality lagrange multipliers'});
legend;

% Plot lower bound multipliers
subplot(3, 2, 5);
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
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the lower bound';'inequality lagrange multipliers'});
legend;

% Plot upper bound multipliers
subplot(3, 2, 6);
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
set(plotChart, {'DisplayName'}, {'Original'; 'Retrieved'})
set(plotChart, {'LineWidth'}, {2; 1.5});
set(plotChart, {'LineStyle'}, {'-'; '-.'});
xticks(tickLocations);
xticklabels(tickNames);
xtickangle(0);
ylabel('Values of ineq mult');
title({'Investigating the upper bound';'inequality lagrange multipliers'});
legend;

