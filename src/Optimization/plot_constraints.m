
function plot_constraints(x,itpParam,modelParam,liftParam,optParam,optimoptions)
%PLOT_CONSTRAINTS plots all the constraints of the problem that are passed.
%
%   PLOT_CONSTRAINTS(x,itpParam,modelParam,liftParam,optParam,optimoptions)

% Compute nonlinear constraints
[C, Ceq, constraintInfoNL] = constraintFunctions(x, itpParam,modelParam,liftParam);

% Apply multipliers to constraints
C = constraintsApplyMultipliersForTolerance(C, constraintInfoNL.Inequalities, optParam);
Ceq = constraintsApplyMultipliersForTolerance(Ceq, constraintInfoNL.Equalities, optParam);

% Generate linear matrices
[A,b,Aeq,beq] = generateLinearConstraints(itpParam,optParam,modelParam,liftParam);

% Compute linear constraints
LC = A*x'-b;
LCeq = Aeq*x'-beq;

% Nonlinear inequality constraints
if ~isequal(C, [])
    figure;
    hold on;
    plot(C, 'DisplayName', 'NLC');
    
    % Plot tolerance if passed
    linear = 1:length(C);
    if ~isequal(optimoptions, [])
        One = ones(1, length(C));
        plot(linear, One*optimoptions.ConstraintTolerance, '--', 'Color', [0 0 0], 'DisplayName', 'UpTol');
    end
    
    % Plot active constraints
    plot(linear(C > optimoptions.ConstraintTolerance), C(C > optimoptions.ConstraintTolerance), 'ro', 'DisplayName', 'Active');
    
    % Plot constraint names
    sum = 0;
    numtypes = length(constraintInfoNL.Inequalities);
    for ii = 1 : numtypes
        xpos = (sum+constraintInfoNL.Inequalities(ii).Amount) * ones(1, 2);
        ypos = [min(C), max(max(C), optimoptions.ConstraintTolerance)];
        plot(xpos, ypos, 'r--', 'HandleVisibility', 'Off');
        text(sum+1, optimoptions.ConstraintTolerance-0.1*diff(ypos), constraintInfoNL.Inequalities(ii).Description);
        sum = sum + constraintInfoNL.Inequalities(ii).Amount;
    end
    xlabel('Constraint order');
    ylabel('Constraint value');
    legend;
    grid;
    title('Nonlinear inequality');
end

% Nonlinear equality constraints
if ~isequal(Ceq, [])
    figure;
    hold on;
    plot(Ceq, 'DisplayName', 'NLCeq');
    
    % Plot tolerance if passed
    linear = 1:length(Ceq);
    if ~isequal(optimoptions, [])
        One = ones(1, length(Ceq));
        plot(linear, One*optimoptions.ConstraintTolerance, '--', 'Color', [0 0 0], 'DisplayName', 'UpTol');
        plot(linear, -One*optimoptions.ConstraintTolerance, '--', 'Color', [0 0 0], 'DisplayName', 'LoTol');
    end
    
    % Plot active constraints
    plot(linear(Ceq > optimoptions.ConstraintTolerance | Ceq < -optimoptions.ConstraintTolerance), ...
         Ceq(Ceq > optimoptions.ConstraintTolerance | Ceq < -optimoptions.ConstraintTolerance), 'ro', 'DisplayName', 'Active');
     
    sum = 0;
    numtypes = length(constraintInfoNL.Equalities);
    for ii = 1 : numtypes
        xpos = (sum+constraintInfoNL.Equalities(ii).Amount) * ones(1, 2);
        ypos = [min(min(Ceq), -optimoptions.ConstraintTolerance), max(max(Ceq), optimoptions.ConstraintTolerance)];
        plot(xpos, ypos, 'r--', 'HandleVisibility', 'Off');
        text(sum+1, optimoptions.ConstraintTolerance-0.1*diff(ypos), constraintInfoNL.Equalities(ii).Description);
        sum = sum + constraintInfoNL.Equalities(ii).Amount;
    end
    xlabel('Constraint order');
    ylabel('Constraint value');
    legend;
    grid;
    title('Nonlinear equality');
end

% Linear inequality constraints
if ~isequal(LC, [])
    figure;
    hold on;
    plot(LC, 'DisplayName', 'LC');
    % Plot tolerance if passed
    if ~isequal(optimoptions, [])
        linear = 1:length(LC);
        One = ones(1, length(LC));
        plot(linear, One*optimoptions.ConstraintTolerance, '-', 'Color', [0.3 1 0.3], 'DisplayName', 'UpTol');
    end    
    xlabel('Constraint order');
    ylabel('Constraint value');
    legend;
    grid;
    title('Linear inequality');
end

% Linear equality constraints
if ~isequal(LCeq, [])
    figure;
    hold on;
    plot(LCeq, 'DisplayName', 'LCeq');
    % Plot tolerance if passed
    if ~isequal(optimoptions, [])
        linear = 1:length(LCeq);
        One = ones(1, length(LCeq));
        plot(linear, One*optimoptions.ConstraintTolerance, '-', 'Color', [0.3 1 0.3], 'DisplayName', 'UpTol');
        plot(linear, -One*optimoptions.ConstraintTolerance, '-', 'Color', [0.3 0.1 0.3], 'DisplayName', 'LoTol');
    end    
    xlabel('Constraint order');
    ylabel('Constraint value');
    legend;
    grid;
    title('Linear equality');
end

end

