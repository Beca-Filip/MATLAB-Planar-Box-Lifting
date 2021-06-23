function COM = COM_nDOF_Cell(q,L,M,CMP)
%COM_NDOF_CELL implements the forward kinematic equations of the center
%of mass of a nDOF planar manipulator for a vectorized input of joint
%angles. Returns the coordinate matrix C of the center of mass along
%the trajectory.
%
%   COM = COM_NDOF_CELL(q, L, M, CMP) takes in the matrix of joint angles
%   (n x Number of samples) alongside the nD vector of segment lengths, nD 
%   vector of normalized segment masses M, and the matrix of center of mass
%   positions CMP, that is of size (n x Number of segments) and returns COM
%   (3 x Number of samples).
%   COM contains in its rows the X and Y position of the overall center of
%   mass across all time samples.
%   Matrix CMP contains in its columns the XYZ position of a segments
%   center of mass with respect to the coordinate system attached to its
%   proximal end in accordance with the modified Denavit-Hartenberg
%   convention.
    
% Exctract useful constants
n = size(q, 1);
N = size(q, 2);

% Perform COM forward kinematics
COM = cell(1, N);        % Prealocate output

% For all samples
for ii = 1 : N
    % Create list of castesian joint rotations
    R = cell(1, n);
    % Create list of cartesian joint position
    P = cell(1, n);
    % Create list of cartesian COM positions
    C = cell(1, n);
    
    % For all joints get the rotations and positions
    for jj = 1 : n
        % Rotation is the sum of the rotations of all previous joints
        R{jj} = Rotz(sum(q(1:jj, ii)));
        % Position is rotated segment length added to the previous joint
        % position
        if jj == 1
            P{jj} = R{jj} * [L(jj);0;0];
        else
            P{jj} = P{jj-1} + R{jj} * [L(jj);0;0];
        end
        % Center of mass positions
        if jj == 1
            C{jj} = R{jj} * CMP(:, jj);
        else
            C{jj} = P{jj-1} + R{jj} * CMP(:, jj);
        end
    end
    
    % Overall position of the COM
    for jj = 1 : n
        if jj == 1
            COM{ii} = M(jj) * C{jj};
        else
            COM{ii} = COM{ii} + M(jj) * C{jj};
        end
    end
end

end