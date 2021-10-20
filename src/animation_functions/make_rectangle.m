function [Xr, Yr] = make_rectangle(theta, x, y, bw, bh, varargin)
%MAKE_RECTANGLE calculates the coordinates of the vertices of a rotated
%rectangle.
%
%   [Xr, Yr] = MAKE_RECTANGLE(theta, x, y, bw, bh) returns the Xr and
%   Yr coordinates of the vertices of a rectangle of width bw and 
%   height bh, if the "lower-left" vertex has coordinates x and y, the 
%   rectangle has orientation theta.
%
%   [Xr, Yr] = MAKE_RECTANGLE(theta, x, y, bw, bh, options) allows to add
%   options such as:
%   - offset_X, offset_Y : offsets the box w.r.t. to the global frame with
%   origin at x,y.
%   - offset_from : from which point is the offset expressed (e.g.
%   'center', 'lower-left' (corner), or 'lower-center' (center of lower
%   side); default 'center')
%   - rotation_frame : either 'global' or 'local' (i.e. rotation after the
%   translation or before the offset)
    
    if nargin > 5
        options = varargin{1};
    else
        options = [];
    end
    
    % Check inputs and assign default values
    if isfield(options, 'offset_X'); offset_X = options.offset_X; else; offset_X = 0; end
    if isfield(options, 'offset_Y'); offset_Y = options.offset_Y; else; offset_Y = 0; end
    if isfield(options, 'offset_from'); offset_from = lower(options.offset_from); else; offset_from = 'center'; end
    if isfield(options, 'rotation_frame'); rotation_frame = lower(options.rotation_frame); else; rotation_frame = 'global'; end
    
    % Check different types of offset
    if isequal(offset_from, 'center')
        offset_adjustment_X = 0;
        offset_adjustment_Y = 0;
    end
    if isequal(offset_from, 'lower-left')
        offset_adjustment_X = +bw/2;
        offset_adjustment_Y = +bh/2;
    end
    if isequal(offset_from, 'lower-center')
        offset_adjustment_X = 0;
        offset_adjustment_Y = +bh/2;
    end
    
    % Create the 2D box in the XY plane centered around (0,0,0)
    B = [[-bw/2, bw/2, bw/2, -bw/2]; [-bh/2, -bh/2, bh/2, bh/2]; zeros(1, 4)];
    
    % If the rotation frame is global the offset must be applied before the
    % rotation and is itself non-rotated    
    if isequal(rotation_frame, 'global')
        B(1, :) = B(1, :) + offset_X + offset_adjustment_X;
        B(2, :) = B(2, :) + offset_Y + offset_adjustment_Y;
    end

    B = rotz(rad2deg(theta)) * B;
    
    % If the rotation frame is local, the offset must be applied after the
    % rotation
    if ~isequal(rotation_frame, 'global') % if it's 'local'
        B(1, :) = B(1, :) + offset_X + offset_adjustment_X;
        B(2, :) = B(2, :) + offset_Y + offset_adjustment_Y;
    end
    
    Xr = B(1, :) + x;
    Yr = B(2, :) + y;
end