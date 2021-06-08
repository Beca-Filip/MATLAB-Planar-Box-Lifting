function C = constraintsApplyMultipliersForTolerance(C, Info, optParam)
%CONSTRAINTSAPPLYMULTIPLIERSFORTOLERANCE multiplies constraint values in C 
%by the multipliers stocked in the optParam structure, depending on their 
%type and number given in the Info structure.

% Initialize a counter
cnt = 0;

% For each type of constraint described in info
for ii = 1 : length(Info)
    
    % Take the constraints corresponding to the described type, and
    % multiply them by the multiplier stored inside the optimization
    % parameters
    C(cnt + 1 : cnt + Info(ii).Amount) = ...
    optParam.(['Mul' Info(ii).Description]) * C(cnt + 1 : cnt + Info(ii).Amount);

    % Update the count variable
    cnt  = cnt + Info(ii).Amount;
end
end