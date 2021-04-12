clear all;
%COSTFUNCTIONNORMALIZATIONSCRIPT is designed to find individual cost
%function extrema to be able to perform normalization.

% Number of cost functions
Ncf = 4;

% Create arrays to store extrema
CostFunctionMinima = zeros(1, Ncf);
CostFunctionMaxima = zeros(1, Ncf);

% Opt Details structure for unsuccessful optimizations
opt_det = {};

% For each cost function
for cf = 1 : Ncf
    
    % Initialize weights of the compound CF to zeros
    outsideWeights = zeros(1, Ncf);
    
    %% Find Minima
    % Intend to minimize that CF by setting its weight to 1 in the compound
    % CF
    outsideWeights(cf) = 1;
    
    % Run optimization script
    Squatting_Motion_Optimization;
    
    % If the exitflag ef_star is not 1 (unsuccessful optimization)
    if ef_star ~= 1
        fprintf(['Unsuccessful minimization of the ' num2str(cf) '. cost function.\n']);
        fprintf(['Exitflag was :' num2str(ef_star) '.\n']);
    end
    
    % Save opt details to check later
    opt_det{end+1} = struct(...
    "cf", cf,...
    "type", "min",...
    "x", x_star,...
    "f", f_star,...
    "ef", ef_star,...
    "out", out_star,...
    "lambda", lbd_star,...
    "grad", grad_star,...
    "hess", hess_star...
    );
    
    % Retrieve minimum function value stored in value f_star inside
    % Squatting_Motion_Optimization script
    CostFunctionMinima(cf) = f_star;
    
    %% Find Maxima
    % Intend to maximize that CF by setting its weight to -1 in the
    % compound CF
    outsideWeights(cf) = -1;
    
    % Run optimization script
    Squatting_Motion_Optimization;
    
    % Check if the exitflag ef_star is 1 (successful optimization)
    if ef_star ~= 1
        fprintf(['Unsuccessful minimization of the ' num2str(cf) '. cost function.\n']);
        fprintf(['Exitflag was :' num2str(ef_star) '.\n']); 
    end 
    
    % Save opt details to check later
    opt_det{end+1} = struct(...
    "cf", cf,...
    "type", "max",...
    "x", x_star,...
    "f", f_star,...
    "ef", ef_star,...
    "out", out_star,...
    "lambda", lbd_star,...
    "grad", grad_star,...
    "hess", hess_star...
    );
    
    % Retrieve maximum function value stored in value f_star inside
    % Squatting_Motion_Optimization script
    CostFunctionMaxima(cf) = f_star;   
    
end

% Invert maxima, since fmax = -(min -f)
CostFunctionMaxima = -CostFunctionMaxima;

% Save results for examination later
save("premilinary.mat", "CostFunctionMinima", "CostFunctionMaxima", "opt_det");