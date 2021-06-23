function dJ = COM_dJacobian_nDOF_Sample(q, dq, L, M, CMP)
%COM_DJACOBIAN_NDOF_SAMPLE calculates the time derivative of the jacobian 
%of the COM FKM for a single sample.
%
%   dJ = COM_DJACOBIAN_NDOF_SAMPLE(q, dq, L, M, CMP)

% Extract useful constants
n = size(q, 1); % Num Joints

% Initialize to empty jac
dJ = [];

% For each joint
sumx = 0;
sumy = 0;
for jj = n : -1 : 1
    
    if jj ~= n
        sumx = sumx + [ones(1, jj) zeros(1, n-jj)] * sum(M(jj+1:end)) * ( -L(jj)*cos(sum(q(1:jj, :))) );
        sumy = sumy + [ones(1, jj) zeros(1, n-jj)] * sum(M(jj+1:end)) * ( -L(jj)*sin(sum(q(1:jj, :))) );
    end
    
    sumx = sumx + [ones(1, jj) zeros(1, n-jj)] * M(jj) * (-cos(sum(q(1:jj, :))) * CMP(1, jj) + sin(sum(q(1:jj, :))) * CMP(2, jj));
    sumy = sumy + [ones(1, jj) zeros(1, n-jj)]* M(jj) * (-sin(sum(q(1:jj, :))) * CMP(1, jj) - cos(sum(q(1:jj, :))) * CMP(2, jj));
    
    dJ = [...
            [
            sumx*dq;
            sumy*dq;
            ], ...
            dJ
        ];
end
end