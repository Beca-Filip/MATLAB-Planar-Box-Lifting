function T = FKM_nDOF_Tensor(q,L)
%FKM_nDOF_TENSOR implements the forward kinematic equations of an n-DOF
%planar manipulator on a vectorized input of joint angles. Returns the 
%transformation matrices of the end effector and all segments along the
%way. 
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   T = FKM_nDOF_TENSOR(q, L) takes in the matrix of joint angles (Number 
%   of Joints x Number of samples) alongside the 3D vector of segment
%   lengths and returns T (4 x 4 x Number of Joints + 1 x Number of
%   samples).
    
% Exctract useful constants
n = size(q, 1); % Number of Joints
N = size(q, 2); % Number of Samples

% Perform forward kinematics
T = zeros(4, 4, n+1, N);  % Prealocate output
T(4, 4, :, :) = 1;      % Set the homogeneous coordinate

for ii = 1 : N
    % Compute transformations
    
    % For first n joints, set the rotation part equal to the rotation of 
    % the joint angle around the z axis
    for jj = 1 : n
        T(1:3, 1:3, jj, ii) = Rotz(q(jj,ii));
    end
    
    % For the last joint, the rotation part is the identity matrix
    T(1:3, 1:3, n+1, ii) = eye(3);
    
    % For the last n joints, set the translation along x to be equal to the
    % segment length and then multiply the transformation matrix by the
    % previous one
    for jj = 2 : n+1
        % Set translation along x
        T(1, 4, jj, ii) = L(jj-1);
        
        % Multiply by transformation of previous segment
        T(:, :, jj, ii) = T(:, :, jj-1, ii) * T(:, :, jj, ii);        
    end
end

end

