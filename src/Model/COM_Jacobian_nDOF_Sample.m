function J = COM_Jacobian_nDOF_Sample(q, L, M, CMP)
%COM_JACOBIAN_NDOF_SAMPLE calculates the jacobian of the COM FKM for a 
%single sample.
%
%   J = COM_JACOBIAN_NDOF_SAMPLE(q, L, M, CMP)

% Extract useful constants
n = size(q, 1); % Num Joints

% Initialize to empty jac
J = [];

% For each joint
sumx = 0;
sumy = 0;
for jj = n : -1 : 1
    
    if jj ~= n
        sumx = sumx + sum(M(jj+1:end)) * ( -L(jj)*sin(sum(q(1:jj, :))) );
        sumy = sumy + sum(M(jj+1:end)) *    L(jj)*cos(sum(q(1:jj, :)))  ;
    end
    
    sumx = sumx + M(jj) * (-sin(sum(q(1:jj, :))) * CMP(1, jj) - cos(sum(q(1:jj, :))) * CMP(2, jj));
    sumy = sumy + M(jj) * (cos(sum(q(1:jj, :))) * CMP(1, jj) - sin(sum(q(1:jj, :))) * CMP(2, jj));
    
    J = [...
            [
            sumx;
            sumy;
            ], ...
            J
        ];
end
end