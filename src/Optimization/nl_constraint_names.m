function [IneqNames, EqNames] = nl_constraint_names(itpParam, modelParam, liftParam)
%NL_CONSTRAINT_NAMES returns a cell array of string names for constraints.


% Get generic constraint info from random evaluation
randominput = rand(1, modelParam.NJoints * itpParam.NumControlPoints);
[~, ~, constraintInfo] = constraintFunctions(randominput,itpParam,modelParam,liftParam);

% Total num of ineq constraints
numIneq = sum([constraintInfo.Inequalities.Amount]);

% Total num of eq constraints
numEq = sum([constraintInfo.Equalities.Amount]);

% Prealocate cell output for inequalities
IneqNames = cell(numIneq, 1);
cnt = 1;
for jj = 1 : length(constraintInfo.Inequalities)
    for ii = 1 : constraintInfo.Inequalities(jj).Amount
        IneqNames{cnt} = constraintInfo.Inequalities(jj).Description;
        cnt = cnt + 1;
    end
end

% Prealocate cell output for equalities
EqNames = cell(numEq, 1);
cnt = 1;
for jj = 1 : length(constraintInfo.Equalities)
    for ii = 1 : constraintInfo.Equalities(jj).Amount
        EqNames{cnt} = constraintInfo.Equalities(jj).Description;
        cnt = cnt + 1;
    end
end

end

