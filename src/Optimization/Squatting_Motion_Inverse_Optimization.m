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
load Data_Optimization.mat

%% Get the cost function and constraint gradient matrices in the right form for KKT IO

% Get the cost function and its gradient
[J_star, dJ_star] = fullify(@(x)costFunctionSet(x), x_star);

% Get the constraint functions and its gradients
[C_star, Ceq_star, dC_star, dCeq_star] = fullify(@(x)nonlinearConstr, x_star);

% Reshape all gradient matrices such that their number of rows equals the
% number of rows of the optimization variable
if size(dJ_star, 1) ~= length(x_star)
    dJ_star = dJ_star';
end
if size(dC_star, 1) ~= length(x_star)
    dC_star = dC_star';
end
if size(dCeq_star, 1) ~= length(x_star)
    dCeq_star = dCeq_star';
end

% Get the different constants of interest

% Number of optimization variables
n = length(x_star);
% Number of cost functions
Ncf = size(dJ_star, 2);
% Number of equality constraints
m = size(dCeq_star, 2);
% Number of inequality constraints
p = size(dC_star, 2);

%% Formulate the IOC problem as a constraint least-squares problem
% Notation from the MATLAB lsqlin documentation will be adopted

% Get Stationarity matrix
M_stat = [dJ_star, dCeq_star, dC_star];

% Get Complementarity matrix
M_compl = [zeros(p, Ncf), zeros(p, m), diag(C_star)];

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

figure;

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

%% Compare found cost function coefficient values to stored coefficient values

figure;
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

%% Check if columns of the matrices are linearly independents

% Check for J_star
[dJ_star_sub, ind_dJ_star] = licols(dJ_star, 1e-3);
size(ind_dJ_star)
% Check for matrix C
[C_sub, ind_C] = licols(C, 1e-3);
size(ind_C)