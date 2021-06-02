function COP = COP_6DOF_Matrix(q,dq,ddq,EW,modelParam)
%COP_6DOF_MATRIX uses the Inverse Dynamics Model in Dyn_6DOF to calculate
%the position of the CoP.
%
%   COP = COP_6DOF_MATRIX(q,dq,ddq,modelParam,EW) returns a (3 x Number of
%   samples) matrix of COP cartesian positions in the model base frame.

% Calculate the wrenches at the base of the robot with the inverse dynamics
% model
[~, EN] = Dyn_6DOF(q,dq,ddq,EW,modelParam);

% Get the position of the COP
% COPx = Mz / Fy
% COPy = 0
% COPz = Mx / Fy
COP = [...
    EN(6, :) ./ EN(2, :);
    zeros(1, size(EN, 2));
    EN(4, :) ./ EN(2, :);
    ];

end

