function Animate_Two_nDOF(q1,L1,q2,L2,Ts,varargin)
%ANIMATE_TWO_NDOF animates two n-DOF planar robots, given their joint 
%position vectors at successive samples, the robots' segment lengths and 
%the sampling rate.
%
%   ANIMATE_TWO_NDOF(q1,L1,q2,L2,Ts) takes in two matrices of joint angles 
%   q1 and q2, their size being (n x Number of samples), two nD vectors of 
%   segment lengths L1 and L2, and the sampling rate Ts in seconds (should 
%   be superior to 0.001s).
%   
%   ANIMATE_TWO_NDOF(q1,L1,q2,L2,Ts,options) allows the possibility of 
%   providing optional arguments such as:
%       - body_thickness1, body_color1, joint_size1, joint_thickness1, 
%       joint_color1: correspond to LineWidth and Color of body 1, 
%       MarkerSize, LineWidth and Color of joints respectively
%       - body_thickness2, body_color2, joint_size2, joint_thickness2, 
%       joint_color2: correspond to LineWidth and Color of body 2,
%       MarkerSize, LineWidth and Color of joints respectively
%       - legend_entry1, legend_entry2, xlabel, ylabel, title : 
%       corresponding plot elements
%       - show_legend, show_grid : to show or not the corresponding plot
%       elements
%       - save_path : where the function should save the animation (without
%       file extension)

% #Input check and initialization of the options structure: options
if nargin > 5
    % Initialize the options variable with the value of the additional argument
    options = varargin{1};
else
    % Create empty options
    options = [];
end

% Check inputs and put default values if not given
if isfield(options, 'body_thickness1'); body_thickness1 = options.body_thickness1; else; body_thickness1 = 2; end
if isfield(options, 'body_color1'); body_color1 = options.body_color1; else; body_color1 = [0 0 0]/255; end
if isfield(options, 'joint_size1'); joint_size1 = options.joint_size1; else; joint_size1 = 10; end
if isfield(options, 'joint_thickness1'); joint_thickness1 = options.joint_thickness1; else; joint_thickness1 = 1; end
if isfield(options, 'joint_color1'); joint_color1 = options.joint_color1; else; joint_color1 = [0 0 0]/255; end
if isfield(options, 'legend_entry1'); legend_entry1 = options.legend_entry1; else; legend_entry1 = 'Robot1'; end

if isfield(options, 'body_thickness2'); body_thickness2 = options.body_thickness2; else; body_thickness2 = 1.8; end
if isfield(options, 'body_color2'); body_color2 = options.body_color2; else; body_color2 = [225 0 0]/255; end
if isfield(options, 'joint_size2'); joint_size2 = options.joint_size2; else; joint_size2 = 10; end
if isfield(options, 'joint_thickness2'); joint_thickness2 = options.joint_thickness2; else; joint_thickness2 = 0.8; end
if isfield(options, 'joint_color2'); joint_color2 = options.joint_color2; else; joint_color2 = [225 0 0]/255; end
if isfield(options, 'legend_entry2'); legend_entry2 = options.legend_entry2; else; legend_entry2 = 'Robot2'; end

if isfield(options, 'xlabel'); xlabel_entry = options.xlabel; else; xlabel_entry = 'X-axis [m]'; end
if isfield(options, 'ylabel'); ylabel_entry = options.ylabel; else; ylabel_entry = 'Y-axis [m]'; end
if isfield(options, 'title'); title_entry = options.title; else; title_entry = 'Animated Planar Robots'; end
if isfield(options, 'show_legend'); show_legend = options.show_legend; else; show_legend = false; end
if isfield(options, 'show_grid'); show_grid = options.show_grid; else; show_grid = true; end

% Check 
if ~isequal(size(q1, 2),size(q2, 2)); error('q1 and q2 must be of same size.'); end

% Extract parameters
N = size(q1, 2);

% Calculate the X and Y coordinates of all the joints across the trajectory
T1 = FKM_nDOF_Tensor(q1, L1);
X1 = squeeze(T1(1, 4, :, :));
Y1 = squeeze(T1(2, 4, :, :));

T2 = FKM_nDOF_Tensor(q2, L2);
X2 = squeeze(T2(1, 4, :, :));
Y2 = squeeze(T2(2, 4, :, :));

hold all;
h_seg1 = plot(X1(:, 1), Y1(:, 1), 'Color', body_color1, 'LineWidth', body_thickness1, 'DisplayName', legend_entry1);
h_jnt1 = plot(X1(:, 1), Y1(:, 1), 'Marker', 'o', 'Color', joint_color1, 'MarkerSize', joint_size1, 'LineWidth', joint_thickness1, 'HandleVisibility', 'Off');

h_seg2 = plot(X2(:, 1), Y2(:, 1), 'Color', body_color2, 'LineWidth', body_thickness2, 'DisplayName', legend_entry2);
h_jnt2= plot(X2(:, 1), Y2(:, 1), 'Marker', 'o', 'Color', joint_color2, 'MarkerSize', joint_size2, 'LineWidth', joint_thickness2, 'HandleVisibility', 'Off');

xlabel(xlabel_entry);
ylabel(ylabel_entry);
title(title_entry);
if show_grid; grid; end
if show_legend; legend; end

% Set orthonormal axes
axis equal
lb = min([min(X1, [], 'all'), min(Y1, [], 'all'), min(X2, [], 'all'), min(Y2, [], 'all')]);
ub = max([max(X1, [], 'all'), max(Y1, [], 'all'), max(X2, [], 'all'), max(Y2, [], 'all')]);
lim = [lb, ub];
xlim(lim);
ylim(lim);

Animate(@(ii)anim_fun(ii,h_seg1,h_jnt1,h_seg2,h_jnt2,X1,Y1,X2,Y2), N, Ts, options);

end

function anim_fun(ii,h_seg1,h_jnt1,h_seg2,h_jnt2,X1,Y1,X2,Y2)

    Planar_nDOF_Callback(ii, h_seg1, X1, Y1);
    Planar_nDOF_Callback(ii, h_jnt1, X1, Y1);
    
    Planar_nDOF_Callback(ii, h_seg2, X2, Y2);
    Planar_nDOF_Callback(ii, h_jnt2, X2, Y2);
end

