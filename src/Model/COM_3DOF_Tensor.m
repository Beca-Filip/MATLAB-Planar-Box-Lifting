function C = COM_3DOF_Tensor(q,L,M,CMP)
%COM_3DOF_TENSOR implements the forward kinematic equations of the center
%of mass of a 3DOF planar manipulator for a vectorized input of joint
%angles. Returns the coordinate matrix C of the center of mass along
%the trajectory.
%
%   C = COM_3DOF_TENSOR(q, L, M, CMP) takes in the matrix of joint angles
%   (3 x Number of samples) alongside the 3D vector of segment lengths, 3D 
%   vector of normalized segment masses M, and the matrix of center of mass
%   positions CMP, that is of size (3 x Number of segments) and returns C
%   (3 x Number of samples).
%   C contains in its rows the X and Y position of the overall center of
%   mass across all time samples.
%   Matrix CMP contains in its columns the XYZ position of a segments
%   center of mass with respect to the coordinate system attached to its
%   proximal end in accordance with the modified Denavit-Hartenberg
%   convention.
    
% Exctract useful constants
N = size(q, 2);
L1 = L(1);
L2 = L(2);
L3 = L(3);
M1 = M(1);
M2 = M(2);
M3 = M(3);

% Perform COM forward kinematics
C = zeros(3, N);        % Prealocate output

for ii = 1 : N
    % Extract useful values
    q1 = q(1, ii);
    q2 = q(2, ii);
    q3 = q(3, ii);
    
    % Rotation matrices
    R1 = Rotz(q1);          % Rotation of 1st segment
    R12 = Rotz(q1+q2);      % Rotation of 2nd segment
    R123 = Rotz(q1+q2+q3);  % Rotation of the 3rd segment
    
    % Position vectors
    P2 = R1 * [L1; 0; 0];           % Position of the 2nd segment
    P3 = P2 + R12 * [L2; 0; 0];     % Position of the 3rd segment
    
    C1 = R1 * CMP(:, 1);            % Position of the 1st COM
    C2 = P2 + R12 * CMP(:, 2);      % Position of the 2nd COM
    C3 = P3 + R123 * CMP(:, 3);     % Position of the 3rd COM
    
    % Overall position of the COM
    C(:, ii) = M1 * C1 + M2 * C2 + M3 * C3;   
end

end

