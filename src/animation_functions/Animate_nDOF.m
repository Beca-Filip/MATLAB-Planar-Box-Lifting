function Animate_nDOF(q,L,Ts,varargin)
%ANIMATE_NDOF animates a n-DOF planar robot, given the joint position
%vectors at successive samples, the robot segment lengths and the sampling
%rate.   
%   ANIMATE_NDOF(q, L, Ts) takes in the matrix of joint angles q, the
%   size being (n x Number of samples), the nD vector of segment lengths L,
%   and the sampling rate Ts in seconds (should be superior to 0.001s).
%   
%   ANIMATE_NDOF(q, L, Ts, options) allows the possibility of providing
%   optional arguments such as:
%       - body_thickness, body_color, joint_size, joint_thickness, 
%       joint_color: correspond to LineWidth and Color of body, MarkerSize,
%       LineWidth and Color of joints respectively
%       - legend_entry, xlabel, ylabel, title : corresponding plot elements
%       - show_legend, show_grid : to show or not the corresponding plot
%       elements
%       - save_path : where the function should save the animation (without
%       file extension)

% #Input check and initialization of the options structure: options
if nargin > 3
    % Initialize the options variable with the value of the additional argument
    options = varargin{1};
else
    % Create empty options
    options = [];
end

% Check inputs and put default values if not given
if isfield(options, 'body_thickness'); body_thickness = options.body_thickness; else; body_thickness = 2; end
if isfield(options, 'body_color'); body_color = options.body_color; else; body_color = [0 0 0]/255; end
if isfield(options, 'joint_size'); joint_size = options.joint_size; else; joint_size = 10; end
if isfield(options, 'joint_thickness'); joint_thickness = options.joint_thickness; else; joint_thickness = 1; end
if isfield(options, 'joint_color'); joint_color = options.joint_color; else; joint_color = [0 0 0]/255; end
if isfield(options, 'legend_entry'); legend_entry = options.legend_entry; else; legend_entry = 'Robot'; end
if isfield(options, 'xlabel'); xlabel_entry = options.xlabel; else; xlabel_entry = 'X-axis [m]'; end
if isfield(options, 'ylabel'); ylabel_entry = options.ylabel; else; ylabel_entry = 'Y-axis [m]'; end
if isfield(options, 'title'); title_entry = options.title; else; title_entry = 'Animated Planar Robot'; end
if isfield(options, 'show_legend'); show_legend = options.show_legend; else; show_legend = false; end
if isfield(options, 'show_grid'); show_grid = options.show_grid; else; show_grid = true; end


% Extract parameters
N = size(q, 2);

% Calculate the X and Y coordinates of all the joints across the trajectory
T = FKM_nDOF_Tensor(q, L);
X = squeeze(T(1, 4, :, :));
Y = squeeze(T(2, 4, :, :));

hold all;
h_seg = plot(X(:, 1), Y(:, 1), 'Color', body_color, 'LineWidth', body_thickness, 'DisplayName', legend_entry);
h_jnt = plot(X(:, 1), Y(:, 1), 'Marker', 'o', 'Color', joint_color, 'MarkerSize', joint_size, 'LineWidth', joint_thickness, 'HandleVisibility', 'Off');
xlabel(xlabel_entry);
ylabel(ylabel_entry);
title(title_entry);
if show_grid; grid; end
if show_legend; legend; end

% Set orthonormal axes
axis equal
lb = min([min(X, [], 'all'), min(Y, [], 'all')]);
ub = max([max(X, [], 'all'), max(Y, [], 'all')]);
lim = [lb, ub];
xlim(lim);
ylim(lim);

Animate(@(ii)anim_fun(ii,h_seg,h_jnt,X,Y), N, Ts, options);

end

function anim_fun(ii,h_seg,h_jnt,X,Y)
    Planar_nDOF_Callback(ii, h_seg, X, Y);
    Planar_nDOF_Callback(ii, h_jnt, X, Y);
end

