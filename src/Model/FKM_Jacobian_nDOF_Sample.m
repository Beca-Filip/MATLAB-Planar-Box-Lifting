function J = FKM_Jacobian_nDOF_Sample(q, L)
%FKM_JACOBIAN_NDOF_SAMPLE calculates the jacobian of the FKM for a single sample.
%
%   J = FKM_JACOBIAN_NDOF_SAMPLE(q, L)

% Extract useful constants
n = size(q, 1); % Num Joints

% Initialize to empty jac
J = [];

% For each joint
sumx = 0;
sumy = 0;
for jj = n : -1 : 1
    sumx = sumx + -L(jj) * sin(sum(q(1:jj, :)));
    sumy = sumy + L(jj) * cos(sum(q(1:jj, :)));
    
    J = [...
            [
            sumx;
            sumy;
            ], ...
            J
        ];
end
end