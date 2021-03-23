function T = FKM_2DOF_Tensor(q,L)
%FKM_2DOF_TENSOR implements the forward kinematic equations of a 2DOF planar
%manipulator on a vectorized input of joint angles. Returns the 
%transformation matrices of the end effector and all segments along the way.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   T = FKM_2DOF_TENSOR(q, L) takes in the matrix of joint angles (2 x Number of
%   samples) alongside the 2D vector of segment lengths and returns T 
%   (4 x 4 x Number of segments + 1 x Number of samples).
    
% Exctract useful constants
N = size(q, 2);
L1 = L(1);
L2 = L(2);

% Perform forward kinematics
T = zeros(4, 4, 3, N);  % Prealocate output
T(4, 4, :, :) = 1;      % Set the homogeneous coordinate

for ii = 1 : N
    % Extract useful values
    q1 = q(1, ii);
    q2 = q(2, ii);
    
    % Compute transformations
    
    % Proximal end of 1st segment
    T(1:3, 1:3, 1, ii) = Rotz(q1);  % Rotation about z
    
    % Proximal end of 2nd segment
    T(1:3, 1:3, 2, ii) = Rotz(q2); % Rotation about z
    T(1, 4, 2, ii) = L1;            % Translation along x
    T(:, :, 2, ii) = T(:, :, 1, ii) * T(:, :, 2, ii);   % Combine with previous transformation
    
    % Distal end of 2nd segment
    T(1:3, 1:3, 3, ii) = eye(3);    % No rotation
    T(1, 4, 3, ii) = L2;            % Translation along x 
    T(:, :, 3, ii) = T(:, :, 2, ii) * T(:, :, 3, ii);   % Combine with previous transformation
end

end

