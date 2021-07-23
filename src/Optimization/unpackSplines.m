function [q,dq,ddq] = unpackSplines(x, modelParam, itpParam, Time)
%UNPACKSPLINES unpacks a vector of control knots along with modelParam and
%itpParam into the trajectory calculated at Time vector.
%
%   [q,dq,ddq] = UNPACKSPLINES(x, modelParam, itpParam, Time)

% Extract ( Reshape is column major )
q_knots = reshape(x, itpParam.NumControlPoints, modelParam.NJoints)';

% Interpolate
for ii = 1 : modelParam.NJoints
    % Get coefficients
    coeff = splineInterpolation2(itpParam.KnotValues, q_knots(ii, :), itpParam.InterpolationOrder, itpParam.BoundaryConditions);
    % Get trajectory, velocity, and acceleration
    qdqddq = splineCoefToTrajectory(itpParam.KnotValues, coeff, Time, 2);
    % Separate them
    q(ii, :) = qdqddq(1, :);
    dq(ii, :) = qdqddq(2, :);
    ddq(ii, :) = qdqddq(3, :);
end

end