function a = FKM_Acceleration_nDOF_Tensor(q, dq, ddq, L)
%FKM_ACCELERATION_NDOF_TENSOR implements the acceleration FKM of an nDOF 
%planar manipulator. The output is the planar 2D velocity of the end-effector
%across time.

% Extract useful constants
N = size(q, 2); % num samples

% Initialize acceleration as empty
a = [];
% For each sample
for ii = 1 : N
    
    % Calculate jacobian at current instant
    J = FKM_Jacobian_nDOF_Sample(q(:, ii), L);
    
    % Calculate the derivative of the jacobian at the current instant
    dJ = FKM_dJacobian_nDOF_Sample(q(:, ii), dq(:, ii), L);
    
    % Add to acceleration
    a = [a, J*ddq(:, ii)+dJ*dq(:, ii)];
end

end