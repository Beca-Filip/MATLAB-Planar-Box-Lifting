function dJ = FKM_dJacobian_nDOF_Sample(q, dq, L)
%FKM_DJACOBIAN_NDOF_SAMPLE calculates the time derivative of the jacobian 
%of the FKM for a single sample.
%
%   dJ = FKM_DJACOBIAN_NDOF_SAMPLE(q, dq, L)

% Extract useful constants
n = size(q, 1); % Num Joints

% Initialize to empty jac
dJ = [];

% For each joint
sumx = 0;
sumy = 0;
for jj = n : -1 : 1
    sumx = sumx + -L(jj) * cos(sum(q(1:jj, :))) * [ones(1, jj), zeros(1, n-jj)];
    sumy = sumy + -L(jj) * sin(sum(q(1:jj, :))) * [ones(1, jj), zeros(1, n-jj)];
    
    dJ = [...
            [
            sumx*dq;
            sumy*dq;
            ], ...
            dJ
        ];
end
end