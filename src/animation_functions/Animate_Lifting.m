function Animate_Lifting(q,L,Ts,varargin)
%ANIMATE_LIFTING animates a n-DOF planar robot, given the joint position
%vectors at successive samples, the robot segment lengths and the sampling
%rate, that lifts a box described by user-defined parameters.
%
%   ANIMATE_LIFTING(q, N, Ts) takes in the matrix of joint angles q, the
%   size being (n x Number of samples), the nD vector of segment lengths L,
%   and the sampling rate Ts in seconds (should be superior to 0.001s), and
%   lifts the box using the default values for the parameters.
%   
%   ANIMATE_LIFTING(q, N, Ts, options) allows the possibility of providing
%   optional arguments such as:
%       - body_thickness, body_color, joint_size, joint_thickness, 
%       joint_color: correspond to LineWidth and Color of body, MarkerSize,
%       LineWidth and Color of joints respectively
%       - legend_entry, xlabel, ylabel, title : corresponding plot elements
%       - show_legend, show_grid : to show or not the corresponding plot
%       elements
%       - save_path : where the function should save the animation (without
%       file extension)
%       - box_width, box_height, box_color, box_legend, box_offset_X,
%       box_offset_Y : related to box
%       - box_offset_from : from which point is the offset expressed
%       ('center' - default, 'lower-left', 'lower-center')
%       - box_rotation_frame : 'global' or 'local' (around the last joint
%       axis or around the box's center axis)
%       - box_lifting_index, box_dropping_index, box_initial_position_X,
%       box_initial_position_Y: when the box is picked up and dropped down
%       and it's initial position

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
if isfield(options, 'title'); title_entry = options.title; else; title_entry = 'Animated Planar Robot Lifting'; end
if isfield(options, 'show_legend'); show_legend = options.show_legend; else; show_legend = false; end
if isfield(options, 'show_grid'); show_grid = options.show_grid; else; show_grid = true; end
if isfield(options, 'box_width'); box_width = options.box_width; else; box_width = 0.25*min(L); end
if isfield(options, 'box_height'); box_height = options.box_height; else; box_height = 0.25*min(L); end
if isfield(options, 'box_color'); box_color = options.box_color; else; box_color = [225, 225, 225]/255; end
if isfield(options, 'box_legend'); box_legend = options.box_legend; else; box_legend = 'Box'; end
if isfield(options, 'box_offset_X'); box_offset_X = options.box_offset_X; else; box_offset_X = 0; end
if isfield(options, 'box_offset_Y'); box_offset_Y = options.box_offset_Y; else; box_offset_Y= 0; end
if isfield(options, 'box_rotation_flag'); box_rotation_flag = options.box_rotation_flag; else; box_rotation_flag = false; end
if isfield(options, 'box_offset_from'); box_offset_from = options.box_offset_from; else; box_offset_from = 'center'; end
if isfield(options, 'box_rotation_frame'); box_rotation_frame = options.box_rotation_frame; else; box_rotation_frame = 'global'; end

% Extract parameters
N = size(q, 2);

% Calculate the X and Y coordinates of all the joints across the trajectory
T = FKM_nDOF_Tensor(q, L);
X = squeeze(T(1, 4, :, :));
Y = squeeze(T(2, 4, :, :));

% Check some more inputs whose default values depend on calculated values
if isfield(options, 'box_lifting_index'); box_lifting_index = options.box_lifting_index; else; box_lifting_index = 1; end
if isfield(options, 'box_dropping_index'); box_dropping_index = options.box_dropping_index; else; box_dropping_index = N; end
if isfield(options, 'box_initial_position_X'); box_initial_position_X = options.box_initial_position_X; else; box_initial_position_X = X(end, 1); end
if isfield(options, 'box_initial_position_Y'); box_initial_position_Y = options.box_initial_position_Y; else; box_initial_position_Y = Y(end, 1); end

% Form box_options structure for box plotting
box_options.offset_X = box_offset_X;
box_options.offset_Y = box_offset_Y;
box_options.offset_from = box_offset_from;
box_options.rotation_frame = box_rotation_frame;

hold all;
h_seg = plot(X(:, 1), Y(:, 1), 'Color', body_color, 'LineWidth', body_thickness, 'DisplayName', legend_entry);
h_jnt = plot(X(:, 1), Y(:, 1), 'Marker', 'o', 'Color', joint_color, 'MarkerSize', joint_size, 'LineWidth', joint_thickness, 'HandleVisibility', 'Off');

[Xbox, Ybox] = make_rectangle(sum(q(:, 1))*box_rotation_flag, box_initial_position_X, box_initial_position_Y, box_width, box_height, box_options);
h_box = patch('XData', Xbox, 'YData', Ybox, 'FaceColor', box_color, 'DisplayName', box_legend);

xlabel(xlabel_entry);
ylabel(ylabel_entry);
title(title_entry);
if show_grid; grid; end
if show_legend; legend; end

% Set orthonormal axes
axis equal
lb = min([min(X, [], 'all'), min(Y, [], 'all')]) - sqrt(box_width.^2 + box_height.^2);
ub = max([max(X, [], 'all'), max(Y, [], 'all')]) + sqrt(box_width.^2 + box_height.^2);
lim = [lb, ub];
xlim(lim);
ylim(lim);

Animate(@(ii)anim_fun(ii,h_seg,h_jnt,h_box,q,X,Y,box_width,box_height,box_options,box_rotation_flag,box_lifting_index,box_dropping_index), N, Ts, options);

end

function anim_fun(ii,h_seg,h_jnt,h_box,q,X,Y,box_width,box_height,box_options,box_rotation_flag,box_lifting_index,box_dropping_index)
    Planar_nDOF_Callback(ii, h_seg, X, Y);
    Planar_nDOF_Callback(ii, h_jnt, X, Y);
    
    if any(ii >= box_lifting_index & ii < box_dropping_index)
        [Xbox, Ybox] = make_rectangle(sum(q(:, ii))*box_rotation_flag, X(end, ii), Y(end, ii), box_width, box_height, box_options);
        h_box.XData = Xbox;
        h_box.YData = Ybox;
    end
end

