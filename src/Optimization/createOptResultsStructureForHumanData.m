load ../../data/Input-Data/Subject1_Filip_Segmented.mat
load ../../data/Output-Data/Optimization-Results/OptResults_Feasible_50CP.mat

resultnames=fieldnames(OptResults.Results);

for ii = 1 : length(resultnames)
    OptResults.Results = rmfield(OptResults.Results, resultnames{ii});
end

OptResults.Results.x_star = q(:, itpParam.KnotIndices).';
OptResults.Results.x_star = OptResults.Results.x_star(:).';

save('../../data/Output-Data/Optimization-Results/OptResults_HumanData.mat');