function Animate_Lifting(q,L,Ts,N,LiftParam,varargin)
%Animates lifting motion with box and table.

% If additional argument is passed, it's the options
if nargin>5
    aopt=varargin{1};
end

% Get lifting start and end
iLiftOff = floor(N * LiftParam.PercentageLiftOff)+1;
iDropOff = floor(N* LiftParam.PercentageDropOff);

% Get the forward kinematic model
T = FKM_nDOF_Tensor(q, L);

% Get wrists transformation
Tw = squeeze(T(:,:,end,:));

% Get wrist position
pw = squeeze(Tw(1:3, 4, :)).'; % row vectors

% Get Box COM position
% BoxCOM = squeeze(Tw(1:3, 4, :)) - LiftParam.BoxToWristVectorDuringLift.';
% BoxCOM = BoxCOM.';

% Create animation options structure
aopt.bgrPlot = @()background(LiftParam.TableRectangle);
aopt.handleInits = {@()handle_inits(LiftParam)};
aopt.callback = @(ii, handle) animate_callbacks(ii, handle, iLiftOff, iDropOff, diff(pw));

% Animate
Animate_nDOF(q,L,Ts,aopt);

end

% Background plotting function for table rectangle
function background(TableRectangle)
rectangle('Position', TableRectangle, 'EdgeColor', [0 0 0]);
end

% Initialize handle for box
function h = handle_inits(LiftParam)
    h = rectangle('Position', LiftParam.BoxRectangleInitial);
end

% function animate_callbacks(ii, handle, iLiftOff, iDropOff, LiftParam, BoxCOM)
function animate_callbacks(ii, handle, iLiftOff, iDropOff, wristdiff)
    if ii > iLiftOff && ii <= iDropOff
%         handle.Position = LiftParam.BoxRectangle(BoxCOM(ii, :));
        handle.Position(1:2) = handle.Position(1:2) + wristdiff(ii-1, 1:2);
%         handle.Position(1:2) = LiftParam.BoxRectangle(1:2) + wristdiff(ii, 1:2);
    end
end