function a_COM = COM_Acceleration_nDOF_Tensor(q,dq,ddq,L,M,CMP)
%COM_ACCELERATION_NDOF_TENSOR implements the forward kinematic acceleration
%equations of the center of mass of a nDOF planar manipulator for a 
%vectorized input of joint angles, velocities and accelerations. It returns
%the acceleration matrix a_COM of the center of mass.
%
%   a_COM = COM_ACCELERATION_NDOF_TENSOR(q, dq, ddq, L, M, CMP) takes in 
%   the matrix of joint angles, velocities and accelerations (n x Number of
%   samples) alongside the nD vector of segment lengths, nD vector of 
%   normalized segment masses M, and the matrix of segment center of mass 
%   positions CMP, that is of size (n x Number of segments) and returns COM
%   planar acceleration (2 x Number of samples).
%   COM contains in its rows the X and Y accelerations of the overall 
%   center of mass across all time samples.
%   Matrix CMP contains in its columns the XYZ position of a segments
%   center of mass with respect to the coordinate system attached to its
%   proximal end in accordance with the modified Denavit-Hartenberg
%   convention.
    
% Exctract useful constants
n = size(q, 1);
N = size(q, 2);

% Perform COM forward kinematics
a_COM = [];        % Prealocate output

% For all samples
for ii = 1 : N
    
    % Get the jacobian
    J = COM_Jacobian_nDOF_Sample(q(:, ii), L, M, CMP);
    
    % Get the derivative of the jacobian
    dJ = COM_dJacobian_nDOF_Sample(q(:, ii), dq(:, ii), L, M, CMP);
    
    % Get the acceleration
    a_COM = [a_COM, dJ*dq(:, ii)+J*ddq(:, ii)];
end

end