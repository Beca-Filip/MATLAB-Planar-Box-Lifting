function T = FKM_2DOF_Cell(q,L)
%FKM_2DOF_CELL implements the forward kinematic equations of a 2DOF planar
%manipulator on a vectorized input of joint angles. Returns the 
%transformation matrices of the end effector and all segments along the way
%within a cell array of size (Number of segments + 1 x Number of samples)
%and whose elements are 4x4 matrices.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   T = FKM_2DOF_CELL(q, L) takes in the matrix of joint angles (2 x Number
%   of samples) alongside the 2D vector of segment lengths and returns T, a
%   (Number of segments + 1 x Number of samples) cell array containing 4x4
%   matrices inside it.
    
% Exctract useful constants
N = size(q, 2);
L1 = L(1);
L2 = L(2);

% Perform forward kinematics
T = cell(size(q, 1)+1, N);  % Prealocate output

for ii = 1 : N
    % Extract useful values
    q1 = q(1, ii);
    q2 = q(2, ii);
    
    % Compute transformations
    
    % Proximal end of 1st segment
    T{1, ii} = [Rotz(q1), zeros(3, 1); zeros(1, 3), 1];  % Rotation about z
    
    % Proximal end of 2nd segment
    T{2, ii} = [Rotz(q2), [L1; zeros(2, 1)]; zeros(1, 3), 1]; % Rotation about z and translation along x
    T{2, ii} = T{1, ii} * T{2, ii}; % Combine with previous transformation
    
    % Distal end of 2nd segment
    T{4, ii} = [eye(3), [L2; zeros(2, 1)]; zeros(1, 3), 1]; % Translation along x
    T{4, ii} = T{3, ii} * T{4, ii};     % Combine with previous transformation
end

end

