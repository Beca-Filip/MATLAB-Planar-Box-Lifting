%COSTFUNCTIONNORMALIZATIONSCRIPT_HUMAN is designed to find the values of
%the cost functions as calculated for the human trajectories.

% Load segmented trajectories
load ../../data/Input-Data/Subject1_Filip_Segmented.mat

%%
savename = 'CostNormalizationHuman.mat';

% Extract the optimization variables
x_human = q(:, itpParam.KnotIndices).';
x_human = x_human(:).';

% Get cost function value
J_human = costFunctions(x_human,itpParam,modelParam,LiftParam);

%%
CostNormalization = J_human;
save(['../../data/Output-Data/Cost-Normalization/' savename], 'CostNormalization');