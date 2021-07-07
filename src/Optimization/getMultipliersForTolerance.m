function mulvec = getMultipliersForTolerance(Info, optParam)
%GETMULTIPLIERSFORTOLERANCE generates a vector of multipliers of constraint
%function values in C by the multipliers stocked in the optParam structure,
%depending on their type given in the Info structure.

% Initialize a counter
cnt = 0;

% Vector of multipliers
mulvec = [];

% For each type of constraint described in info
for ii = 1 : length(Info)
    
    % Take the constraints corresponding to the described type, and
    % multiply them by the multiplier stored inside the optimization
    % parameters
    mulvec(cnt + 1 : cnt + Info(ii).Amount) = ...
    optParam.(['Mul' Info(ii).Description]) * ones(1, Info(ii).Amount);

    % Update the count variable
    cnt  = cnt + Info(ii).Amount;
end
end