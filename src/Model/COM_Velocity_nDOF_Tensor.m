function v_COM = COM_Velocity_nDOF_Tensor(q,dq,L,M,CMP)
%COM_VELOCITY_NDOF_TENSOR implements the forward kinematic velocity 
%equations of the center of mass of a nDOF planar manipulator for a 
%vectorized input of joint angles and velocities. It returns the velocity
%matrix v_COM of the center of mass.
%
%   v_COM = COM_VELOCITY_NDOF_TENSOR(q, dq, L, M, CMP) takes in the matrix 
%   of joint angles and velocities (n x Number of samples) alongside the nD
%   vector of segment lengths, nD vector of normalized segment masses M, 
%   and the matrix of center of mass positions CMP, that is of size (n x 
%   Number of segments) and returns v_COM (2 x Number of samples).
%   v_COM contains in its rows the X and Y velocities of the overall center
%   of the center mass across all time samples.
%   Matrix CMP contains in its columns the XYZ position of a segments
%   center of mass with respect to the coordinate system attached to its
%   proximal end in accordance with the modified Denavit-Hartenberg
%   convention.
    
% Exctract useful constants
n = size(q, 1);
N = size(q, 2);

% Perform COM forward kinematics
v_COM = [];        % Prealocate output

% For all samples
for ii = 1 : N
    
    % Get the jacobian
    J = COM_Jacobian_nDOF_Sample(q(:, ii), L, M, CMP);
    
    % Get the velocity
    v_COM = [v_COM, J*dq(:, ii)];
end

end