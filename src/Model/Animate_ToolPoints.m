function [Xt, Yt, Ltool] = Animate_ToolPoints(theta, Xe, Ye, tool)
%ANIMATE_TOOLPOINTS helps with the animation of the tool. Calculates the
%plotting positions of the tool.
%
%   [Xt, Yt, Ltool] = ANIMATE_TOOLPOINTS(theta, Xe, Ye, tool) returns tool
%   coordinates of a specified type and specified by parameters, in a
%   (Number of tool coordinates x Number of samples) matrix. Also returns
%   the tool's longest dimension within Ltool.
%       theta - angle of the end effector
%       Xe - x position of the end effector
%       Ye - y position of the end effector
%
%   Example:
%       tool.type = 'hand';
%       tool.length = 1;
%       [Xt, Yt] = ANIMATE_TOOLPOINTS(theta, Xe, Ye, tool) returns a
%       'hand' type tool coordinates to plot. The tool will have width 2
%       and length 1 (Width always 2x height). The coordinates are stored in a 
%       (Number of tool coordinates x Number of samples) matrix. For a hand
%       type tool, the number of tool coordinates is 4 (since we need to plot
%       lines between 4 points to draw a hand tool).
%       
%       tool.type = 'circle';
%       tool.diameter = 3;
%       [Xt, Yt] = ANIMATE_TOOLPOINTS(theta, Xe, Ye, tool)
%       Draws a circle of diameter 3 and renders it with 50 points. Meaning
%       the number of tool coordinates is 50.

% Reshape entries into row vectors
theta = reshape(theta, 1, []);
Xe = reshape(Xe, 1, []);
Ye = reshape(Ye, 1, []);

% Useful constants
N = length(theta);

% Determine type of tool

% 1. If circle tool is specified
if isequal(lower(tool.type), "circle")
    
    % Extract useful constants
    Ltool = tool.diameter;
    Npts = 50;
    
    % Calculate center of the circle at each timestep
    Xcenter = Xe + Ltool/2 * cos(theta);
    Ycenter = Ye + Ltool/2 * sin(theta);
    
    % Create a generic circle around the origin, using parametrized form
    phi = linspace(0, 2*pi, Npts)';
    Xcircle = Ltool / 2 * cos(phi);
    Ycircle = Ltool / 2 * sin(phi);
    
    % Tool points: At each timestep, add the center coordinates to generic
    % circle coordinates
    Xt = Xcenter + repmat(Xcircle, 1, N);
    Yt = Ycenter + repmat(Ycircle, 1, N);
    
% 2. If hand tool is specified or if nothing is specified
else  % elseif isequal(lower(type), "hand")
    
    % Extract useful constants
    Ltool = tool.length;
               
    % Calculate the x coordinates of the left finger, left edge, right edge
    % and right finger
    XLF = Xe + Ltool * (cos(theta) - sin(theta));    
    XLE = Xe - Ltool * sin(theta);
    XRE = Xe + Ltool * sin(theta);
    XRF = Xe + Ltool * (cos(theta) + sin(theta));
    
    % Calculate the y coordinates of the left finger, left edge, right edge
    % and right finger
    YLF = Ye + Ltool * (sin(theta) + cos(theta));
    YLE = Ye + Ltool * cos(theta);
    YRE = Ye - Ltool * cos(theta);
    YRF = Ye + Ltool * (sin(theta) - cos(theta));
    
    % Stack in a single matrix
    Xt = [XLF; XLE; XRE; XRF];
    Yt = [YLF; YLE; YRE; YRF]; 
end

end

