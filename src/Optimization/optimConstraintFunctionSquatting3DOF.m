function [C, Ceq] = optimConstraintFunctionSquatting3DOF(x,itpParam,optTolerance,modelParam)
%OPTIMCONSTRAINTFUNCTIONSQUATTING3DOF implements the constraints on a 3DOF
%model of a human performing a squat.

% Extract useful constants
n = itpParam.NumControlPoints;
t = itpParam.KnotValues;
p = itpParam.InterpolationOrder;
bndcnd = itpParam.BoundaryConditions;

% Unpack optimization variable
q1_knot = x(1:n);
q2_knot = x(n+1:2*n);
q3_knot = x(2*n+1:3*n);


%% Intermediate Computations: Final State Neck and Hip Heights

% Find the final joint angles
qf = [q1_knot(end); q2_knot(end); q3_knot(end)];

% Make a segment length vector
L = [modelParam.L1; modelParam.L2; modelParam.L3];

% Call the appropriate FKM function for the final state
Tf = FKM_3DOF_Cell(qf, L);

% Get final neck height
nhf = Tf{end, 1}(2, 4);

% Get final hip height
hhf = Tf{end-1, 1}(2, 4);

%% Inequality constraints

C = [
    optTolerance.MulFinalConditions * (nhf - modelParam.FinalNeckHeightPercentage * sum(L));
    optTolerance.MulFinalConditions * (hhf - modelParam.FinalHipHeightPercentage * sum(L(1:end-1)));
    ];

%% Equality constraints

Ceq = [];
end

