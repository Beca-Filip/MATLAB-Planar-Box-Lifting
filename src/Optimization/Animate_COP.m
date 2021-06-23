%ANIMATE_COP

% Get external wrenches
EW = getExternalWrenches(q, L, LiftParam);

% Get COP
COP = COP_6DOF_Matrix(q_star, dq_star, ddq_star, modelParam, EW);
% Plot COP
opts.handleInits = {@()plot(0, 0, 'go', 'DisplayName', 'COP')};
opts.callback = @(ii, handle) cop_callback(ii, handle, COP);
% Plot COP bounds
opts.bgrPlot = @() bgr_plot(LiftParam.HeelPosition, LiftParam.ToePosition);

Animate_nDOF(q_star, L, Ts, opts);

function cop_callback(ii, handle, COP)
    handle.XData = COP(1, ii);
end

function bgr_plot(low, high)
    plot(low, 0, 'ro', 'DisplayName', 'CopLo');
    plot(high, 0, 'bo', 'DisplayName', 'CopHi')
end