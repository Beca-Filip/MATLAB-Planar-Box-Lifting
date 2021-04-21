    clear all; close all; clc;

%% Import necessary libs and data

% Add the data to path
addpath("../../data/3DOF/Squat");

% Add model functions
addpath("../Model"); 

% Add the spline functions
addpath("../../libs/splinePack/");  

% Add CASADI library
if ismac                            
    % Code to run on Mac platform
    addpath('../../libs/casadi-osx-matlabR2015a-v3.5.1')
elseif isunix
    % Code to run on Linux platform
    addpath('../../libs/casadi-linux-matlabR2015a-v3.5.1')
elseif ispc
    % Code to run on Windows platform
    addpath('../../libs/casadi-windows-matlabR2016a-v3.5.5')
else
    disp('Platform not supported')
end

% Add generated functions to path
addpath('optimSquattingComputables\');

% Load optimal data
load_filename = 'MinimumTorque_50_ConstraintPoints';
load(['../../data/3DOF/Optimization-Human/Storage_' load_filename '.mat']);

%% Create flags and parameters for the running of this script

% Create a flag for saving graphs
scriptParam.SaveGraphs = false;

% Create a prefix for saving graphs
scriptParam.SavePrefix = [load_filename '_Full_'];

% Create a saving path for graphs
scriptParam.SavePath = '../../data/Produced-Graphs/Inverse-Optimization/';
scriptParam.SavePathFigs = '../../data/Produced-Graphs/Inverse-Optimization/matFigs/';

% Create a file format suffix
scriptParam.SaveFormat = '.png';

%% Get the cost function and constraint gradient matrices

% Extract optimal solution
itpParam = Storage.itpParam;
optParam = Storage.optParam;
modelParam = Storage.modelParam;
x_star = Storage.Results.x_star;

% Get the cost function and its gradient
[J_star, dJ_star] = fullify(@(x)costFunctionSet(x), x_star);

% Get the nonlinear constraint functions and its gradients
[C_star, Ceq_star, dC_star, dCeq_star] = fullify(@(x)nonlinearConstr(x), x_star);

% Get the linear constraint matrices
[A_star, b_star, Aeq_star, beq_star] = optimGenerateLinearConstraintMatricesSquatting3DOF(itpParam, optParam, modelParam);

% Generate upper and lower bounds
One = ones(1, itpParam.NumControlPoints);
lb_doc = [modelParam.JointLimits(1, 1)*One, modelParam.JointLimits(1, 2)*One, modelParam.JointLimits(1, 3)*One];
ub_doc = [modelParam.JointLimits(2, 1)*One, modelParam.JointLimits(2, 2)*One, modelParam.JointLimits(2, 3)*One];

%% Preprocess gradient matrices to prepare them for IOC

% Reshape all gradient matrices such that their number of rows equals the
% number of rows of the optimization variable
% Gradient of cost function
if size(dJ_star, 1) ~= length(x_star)
    dJ_star = dJ_star';
end
% Gradient of nonlinear inequality constraints
if size(dC_star, 1) ~= length(x_star)
    dC_star = dC_star';
end
% Gradient of nonlinear equality constraints
if size(dCeq_star, 1) ~= length(x_star)
    dCeq_star = dCeq_star';
end
% Gradient of linear inequality constraints
if size(A_star, 1) ~= length(x_star)
    A_star = A_star';
end
% Gradient of linear equality constraints
if size(Aeq_star, 1) ~= length(x_star)
    Aeq_star = Aeq_star';
end

% Gradient of lower and upper bounds
A_lb_star = -eye(length(x_star));
b_lb_star = -lb_doc;
A_ub_star = eye(length(x_star));
b_ub_star = ub_doc;

% Append nonlinear, linear and bound inequality gradients into a single matrix
dIneq = [A_star, dC_star, A_lb_star, A_ub_star];

% Append nonlinear and linear equality gradients into a single matrix
dEq = [Aeq_star, dCeq_star];

% Append nonlinear and linear inequality values into a single vector but
% first reshape them into row vectors
rshpIneqLin = reshape(x_star * A_star - reshape(b_star, 1, []), 1, []);
rshpLb = x_star * A_lb_star - b_lb_star;
rshpUb = x_star * A_ub_star - b_ub_star;
Ineq = [rshpIneqLin, reshape(C_star, 1, []), rshpLb, rshpUb];

% Append linear and nonlinear equality values into a single vector, but
% make them row vectors first
rshpEqLin = reshape(x_star * Aeq_star - reshape(beq_star, 1, []), 1, []);
Eq = [rshpEqLin, reshape(Ceq_star, 1, [])];

% Get the different constants of interest

% Number of optimization variables
n = length(x_star);
% Number of cost functions
Ncf = size(dJ_star, 2);
% Number of equality constraints
m = size(dEq, 2);
% Number of inequality constraints
p = size(dIneq, 2);

%% Formulate the IOC problem as a constraint least-squares problem
% Notation from the MATLAB lsqlin documentation will be adopted

% Get Stationarity matrix
M_stat = [dJ_star, dEq, dIneq];

% Get Complementarity matrix
M_compl = [zeros(p, Ncf), zeros(p, m), diag(Ineq)];

% Get Multiplicative matrix
C = [M_stat; M_compl];

% Get Yielding Matrix
d = zeros(n+p, 1);

% There are no inequalities (there are bounds)
A = [];
b = [];

% Only equality is that the sum of cost function parameters must be 1
Aeq = zeros(1, Ncf+m+p);
Aeq(1, 1:Ncf) = 1;
beq = 1;

% Lower bounds exist on cost function parameters and inequality langrange
% multiplicators and are equal to zero (an arbitrary lower bound of lb_lam
% shall be placed upon equality lagrange multipliers to limit the search
% space)
lb_lam = -1e3; 
lb = [zeros(Ncf, 1); -1e3 * ones(m, 1); zeros(p, 1)];
ub = [];

%% Inverse optimization pipeline

% Get optimal coefficients
[vars_ioc,rn_ioc,res_ioc,ef_ioc,out_ioc,~] = lsqlin(C,d,A,b,Aeq,beq,lb,ub);

% Extract difference quantities
alpha_ioc = vars_ioc(1:Ncf);
lambda_ioc = vars_ioc(Ncf+1 : Ncf+m);
mu_ioc = vars_ioc(Ncf+m+1 : Ncf+m+p);

%% Investigate the residual norm the residual itself

% Create figure handle
fig_residuals = figure;

subplot(1, 3, [1 2])
% Plot residual accross dimension
barValues = res_ioc;
numbars = length(res_ioc);
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['Dim. ' num2str(ii, '%02d')];
end
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations(1:10:end));
xticklabels(barNames(1:10:end));
xtickangle(70);
ylabel('Residual of the Lagrangian along given dimension');
title({'Investigating the ';'residual of the Lagrangian'});

subplot(1, 3, 3)
% Plot a single bar that represents the residual norm
barValues = rn_ioc;
numbars = length(rn_ioc);
barLocations = 1:numbars;
barNames = {'Residual Norm'};
hold on;
barChart = bar(barLocations, barValues);
barChart.FaceColor = 'flat';    % Let the facecolors be controlled by CData
barChart.CData = repmat(linspace(0.2, 0.8, numbars)', 1, 3);     % Set CData
xticks(barLocations);
xticklabels(barNames);
xtickangle(0);
ylabel('Residual norm of the Lagrangian');
title({'Investigating the residual';' norm of the Lagrangian'});

% Save the graph
if scriptParam.SaveGraphs    
    % Save the graphs in given directory
    saveas(fig_residuals, [scriptParam.SavePath, scriptParam.SavePrefix, 'Residuals', scriptParam.SaveFormat]);
    
    % Save the graph in .fig format for eventual later editing
    saveas(fig_residuals, [scriptParam.SavePathFigs scriptParam.SavePrefix 'Residuals.fig']);
end

%% Compare found cost function coefficient values to stored coefficient values

fig_weightretrieval = figure;
% Plot a bars that represent the cost implication
barValues = [optParam.CostFunctionWeights; alpha_ioc'];
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
ylabel('Values of cost function parametrization');
title({'Investigating the';'cost function parametrization'});
legend;

% Save the graph
if scriptParam.SaveGraphs    
    % Save the graphs in given directory
    saveas(fig_weightretrieval, [scriptParam.SavePath scriptParam.SavePrefix 'Retrieval' scriptParam.SaveFormat]);
    
    % Save the graph in .fig format for eventual later editing
    saveas(fig_weightretrieval, [scriptParam.SavePathFigs scriptParam.SavePrefix 'Retrieval.fig']); 
end

%% Check if columns of the matrices are linearly independents

% Check for J_star
[dJ_star_sub, ind_dJ_star] = licols(dJ_star, 1e-3);
% size(ind_dJ_star)
% Check for matrix C
[C_sub, ind_C] = licols(C, 1e-3);
% size(ind_C)
% Print the residual norm
fprintf("The residual norm of inverse optimization is %.4f.\n", rn_ioc);
% Check rank of C
fprintf("The size of the recovery matrix is [%d, %d] while its rank is %d.\n", size(C), rank(C));
fprintf("The condition number of the recovery matrix is %.4f.\n", cond(C));
% Check rank of part of C corresponding to the part of the recovery matrix
% that is relative to the cost functions
fprintf("The size of the cost function part of the recovery matrix is [%d, %d] while its rank is %d.\n", size(C(:, 1:Ncf)), rank(C(:, 1:Ncf)));
fprintf("The condition number of the cost function part of the recovery matrix is %.4f.\n", cond(C(:, 1:Ncf)));


%% Plot the sorted conditioning

% Plot the conditioning
fig_sortedconditioning = figure;

% Get SVD
[U, S, V] = svd(C);

% Plot singular values
subplot(3, 1, 1)
barValues = diag(S)';
numbars = length(diag(S));
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['s_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'SV'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('Singular Values');
title({'Singular values of';'the regressor matrix'});
legend;

% Get the norm of column vectors
colNormC = vecnorm(C);

% Plot the norm of column vectors
subplot(3, 1, 2)
barValues = sort(colNormC, 'descend');
numbars = length(colNormC);
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['|c_{' num2str(ii) '}|'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'col norm'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('Col norms');
title({'Sorted norms of the columns of';'the regressor matrix'});
legend;

% Get QR decomp
[Q, R, P] = qr(C);

% Plot QR values
subplot(3, 1, 3)
barValues = sort(diag(R)', 'descend');
numbars = length(diag(R));
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['r_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'QV'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('R diag elems');
title({'Sorted diagonal elements of R in'; ' QR decomp of the regressor matrix'});
legend;

% Save the graph
if scriptParam.SaveGraphs    
    % Save the graphs in given directory
    saveas(fig_sortedconditioning, [scriptParam.SavePath scriptParam.SavePrefix 'CondSorted' scriptParam.SaveFormat]);
    
    % Save the graph in .fig format for eventual later editing
    saveas(fig_sortedconditioning, [scriptParam.SavePathFigs scriptParam.SavePrefix 'CondSorted.fig']); 
end

%% Plot the unsorted conditioning

% Plot the conditioning
fig_unsortedconditioning = figure;

% Get SVD
[U, S, V] = svd(C);

% Plot singular values
subplot(3, 1, 1)
barValues = diag(S)';
numbars = length(diag(S));
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['s_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'SV'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('Singular Values');
title({'Singular values of';'the regressor matrix'});
legend;

% Get the norm of column vectors
colNormC = vecnorm(C);

% Plot the norm of column vectors
subplot(3, 1, 2)
barValues = colNormC;
numbars = length(colNormC);
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['|c_{' num2str(ii) '}|'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'col norm'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('Col norms');
title({'Unsorted norms of the columns of';'the regressor matrix'});
legend;

% Get QR decomp
[Q, R, P] = qr(C);

% Plot QR values
subplot(3, 1, 3)
barValues = diag(R)';
numbars = length(diag(R));
barLocations = 1:numbars;
barNames = {};
for ii = 1 : numbars
    barNames{ii} = ['r_{' num2str(ii) '}'];
end
hold on;
barChart = bar(barLocations, barValues);
set(barChart, {'DisplayName'}, {'QV'})
xticks(barLocations(1:ceil(end/20):end));
xticklabels(barNames(1:ceil(end/20):end));
xtickangle(0);
ylabel('R diag elems');
title({'Unsorted diagonal elements of R in'; ' QR decomp of the regressor matrix'});
legend;

% Save the graph
if scriptParam.SaveGraphs    
    % Save the graphs in given directory
    saveas(fig_unsortedconditioning, [scriptParam.SavePath scriptParam.SavePrefix 'CondUnsorted' scriptParam.SaveFormat]);
    
    % Save the graph in .fig format for eventual later editing
    saveas(fig_unsortedconditioning, [scriptParam.SavePathFigs scriptParam.SavePrefix 'CondUnsorted.fig']); 
end