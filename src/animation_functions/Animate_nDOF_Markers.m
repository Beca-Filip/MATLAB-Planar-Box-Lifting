function Animate_nDOF_Markers(q,Markers,L,Ts,varargin)
%ANIMATE_NDOF_MARKERS animates a n-DOF planar robot and the markers 
%associated with its joints, given the joint positions vectors at 
%successive samples, the  marker 3D positions, and the robot segment 
%lengths and the sampling rate.
%
%   ANIMATE_NDOF_MARKERS(q, Markers, L, Ts) takes in the matrix of joint angles q, 
%   the size being (n x Number of samples), the Markers structure 
%   containing the 3D positions of different Markers, the nD vector of 
%   segment lengths L, and the sampling rate Ts in seconds (should be 
%   superior to 0.001s).
%   
%   ANIMATE_NDOF_MARKERS(q, Markers, L, Ts, options) allows the possibility of 
%   providing optional arguments such as:
%       - body_thickness, body_color, joint_size, joint_thickness, 
%       joint_color: correspond to LineWidth and Color of body, MarkerSize,
%       LineWidth and Color of joints respectively
%       - markers_marker, markers_color, markers_separate_legend, 
%       markers_size, markers_thickness, markers_center: sets the Marker
%       shape, the colormap to use (with number of markers elements), a
%       flag indicating whether to label markers separately in legend, the
%       MarkerSize, the LineWidth, and the origin with which we want to
%       express the Markers (e.g. the ankle Markers.BODY.RANK).
%       - legend_entry, xlabel, ylabel, title : corresponding plot elements
%       - show_legend, show_grid : to show or not the corresponding plot
%       elements
%       - save_path : where the function should save the animation (without
%       file extension)

% #Input check and initialization of the options structure: options
if nargin > 4
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

% Calculate number of markers
NumMark = 0;
Msetnames = fieldnames(Markers);
for ii = 1 : length(Msetnames)
    NumMark = NumMark + length(fieldnames(Markers.(Msetnames{ii})));
end

% Check Marker options
if isfield(options, 'markers_marker'); markers_marker = options.markers_marker; else; markers_marker = 'o'; end
if isfield(options, 'markers_color'); markers_color = options.markers_color; else; markers_color = colorcube(NumMark+1); end
if isfield(options, 'markers_separate_legend'); markers_separate_legend = options.markers_separate_legend; else; markers_separate_legend = true; end
if isfield(options, 'markers_size'); markers_size = options.markers_size; else; markers_size = 10; end
if isfield(options, 'markers_thickness'); markers_thickness = options.markers_thickness; else; markers_thickness = 1; end
if isfield(options, 'markers_center'); markers_center = options.markers_center; else; markers_center = zeros(N, 3); end

%  Load the coordinates for Markers
Xm = zeros(NumMark, N);
Ym = zeros(NumMark, N);
MarkerLabels = cell(NumMark, 1);
CurrMark = 1;
for ii = 1 : length(Msetnames)
    Mnames = fieldnames(Markers.(Msetnames{ii}));
    for jj = 1 : length(Mnames)
        Xm(CurrMark, :) = (Markers.(Msetnames{ii}).(Mnames{jj})(1:N, 1) - markers_center(1:N, 1))';
        Ym(CurrMark, :) = (Markers.(Msetnames{ii}).(Mnames{jj})(1:N, 2) - markers_center(1:N, 2))';
        MarkerLabels{CurrMark} = [Msetnames{ii}, '.', Mnames{jj}];
        CurrMark = CurrMark + 1;
    end
end

% Calculate the X and Y coordinates of all the joints across the trajectory
T = FKM_nDOF_Tensor(q, L);
X = squeeze(T(1, 4, :, :));
Y = squeeze(T(2, 4, :, :));

hold all;
h_seg = plot(X(:, 1), Y(:, 1), 'Color', body_color, 'LineWidth', body_thickness, 'DisplayName', legend_entry);
h_jnt = plot(X(:, 1), Y(:, 1), 'Marker', 'o', 'LineStyle', 'None', 'Color', joint_color, 'MarkerSize', joint_size, 'LineWidth', joint_thickness, 'HandleVisibility', 'Off');

if markers_separate_legend
    h_mark = gobjects(NumMark, 1);
    for ii = 1 : NumMark
        h_mark(ii) = plot(Xm(ii, 1), Ym(ii, 1), 'Marker', markers_marker, 'LineStyle', 'None', 'Color', markers_color(ii, :), 'MarkerSize', markers_size, 'LineWidth', markers_thickness, 'DisplayName', MarkerLabels{ii});
    end
else
    h_mark = plot(Xm(:, 1), Ym(:, 1), 'Marker', markers_marker, 'LineStyle', 'None', 'Color', markers_color(1, :), 'MarkerSize', markers_size, 'LineWidth', markers_thickness, 'DisplayName', 'Markers');
end

xlabel(xlabel_entry);
ylabel(ylabel_entry);
title(title_entry);
if show_grid; grid; end
if show_legend; legend; end

% Set orthonormal axes
axis equal
lb = min([X(:); Y(:); Xm(:); Ym(:)]);
ub = max([X(:); Y(:); Xm(:); Ym(:)]);
lim = [lb, ub];
xlim(lim);
ylim(lim);

Animate(@(ii)anim_fun(ii,h_seg,h_jnt,h_mark,X,Y,Xm,Ym,markers_separate_legend), N, Ts, options);

end

function anim_fun(ii,h_seg,h_jnt,h_mark,X,Y,Xm,Ym,markers_separate_legend)
    Planar_nDOF_Callback(ii, h_seg, X, Y);
    Planar_nDOF_Callback(ii, h_jnt, X, Y);
    
    if markers_separate_legend
        for nn = 1 : length(h_mark)
            h_mark(nn).XData = Xm(nn, ii);
            h_mark(nn).YData = Ym(nn, ii);
        end
    else
        h_mark.XData = Xm(:, ii);
        h_mark.YData = Ym(:, ii);
    end
end

