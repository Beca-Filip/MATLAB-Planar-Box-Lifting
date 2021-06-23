function v = FKM_Velocity_nDOF_Tensor(q, dq, L)
%FKM_VELOCITY_NDOF_TENSOR implements the velocity FKM of an nDOF planar
%manipulator. The output is the planar 2D velocity of the end-effector
%across time.

% Extract useful constants
N = size(q, 2); % num samples

% Initialize velocity as empty
v = [];
% For each sample
for ii = 1 : N
    
    % Calculate jacobian at current instant
    J = FKM_Jacobian_nDOF_Sample(q(:, ii), L);
    
    % Add to velocity
    v = [v, J*dq(:, ii)];
end

end