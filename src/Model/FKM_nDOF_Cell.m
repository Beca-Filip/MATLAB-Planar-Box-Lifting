function T = FKM_nDOF_Cell(q,L)
%FKM_NDOF_CELL implements the forward kinematic equations of a 3DOF planar
%manipulator on a vectorized input of joint angles. Returns the 
%transformation matrices of the end effector and all segments along the way
%within a cell array of size (Number of segments + 1 x Number of samples)
%and whose elements are 4x4 matrices.
%Assumes the proximal Denavit-Hartenberg assignement of coordinate systems.
%
%   T = FKM_NDOF_CELL(q, L) takes in the matrix of joint angles (3 x Number
%   of samples) alongside the 3D vector of segment lengths and returns T, a
%   (Number of segments + 1 x Number of samples) cell array containing 4x4
%   matrices inside it.
    
% Exctract useful constants
n = size(q, 1); % Number of Joints
N = size(q, 2); % Number of Samples

% Perform forward kinematics
T = cell(n+1, N);  % Prealocate output

for ii = 1 : N
    % Compute transformations
    
    % For first n joints, set the rotation part equal to the rotation of 
    % the joint angle around the z axis
    for jj = 1 : n
        T{jj, ii}= [Rotz(q(jj,ii)) [0;0;0]; 0 0 0 1];
    end
    
    % For the last joint
    T{n+1, ii} = eye(4);    % Initialize to 4x4 Identity
                            % Rotation part is already identity
    
    % For the last n joints, set the translation along x to be equal to the
    % segment length and then multiply the transformation matrix by the
    % previous one
    for jj = 2 : n+1
        % Set translation along x
        T{jj, ii}(1, 4) = L(jj-1);
        
        % Multiply by transformation of previous segment
        T{jj, ii} = T{jj-1, ii} * T{jj, ii};        
    end    
end

end

