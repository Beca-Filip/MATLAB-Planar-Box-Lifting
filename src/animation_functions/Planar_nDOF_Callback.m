function Planar_nDOF_Animation_Callback(ii, h, X, Y)
%PLANAR_NDOF_CALLBACK takes in the X and Y coordinates of the joint
%locations of an nDOF planar manipulator accross time, and updates the
%handle h with them.
    h.XData = X(:, ii);
    h.YData = Y(:, ii);
end

