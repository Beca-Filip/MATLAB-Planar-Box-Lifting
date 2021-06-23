%ANIMATE_COM

% Get COM
COM = COM_nDOF_Cell(q,L,M,CMP);
COM = cell2mat(COM);

% Plot COM
opts.handleInits = {@()plot(0, 0, 'r*', 'DisplayName', 'COM')};
opts.callback = @(ii, handle) com_callback(ii, handle, COM);

% Animate_Lifting(q_star, L, Ts, LiftParam, opts);
Animate_nDOF(q_star, L, Ts, opts);

function com_callback(ii, handle, COM)
    handle.XData = COM(1, ii);
    handle.YData = COM(2, ii);
end