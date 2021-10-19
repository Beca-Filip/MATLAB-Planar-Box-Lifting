function compare_spline_interpolations(x1,x2,modelParam,itpParam,Time)
%COMPARE_SPLINE_INTERPOLATIONS compares two trajectories given by their 
%control points.
%
%   e = COMPARE_TRAJECTORIES(x1,x2,itpParam) returns the RMSE between 
%   interpolated trajectories from x1, x2.

% Define rmse 
rmse = @(a,b) sqrt(sum((a-b).^2 / numel(a), 'all'));

% Get trajectories
[q1, dq1, ddq1] = unpackSplines(x1, modelParam, itpParam, Time);
[q2, dq2, ddq2] = unpackSplines(x2, modelParam, itpParam, Time);

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};
% Make 2d vector of ones for plotting
Ones = ones(1, 2);
% How many rows
figcols = 2;
figrows = 3;
% Create figure
figure;
sgtitle({'Comparison of trajectories';...
        ['Total RMSE = ', num2str(rad2deg(rmse(q1, q2)), '%.4f'), 'deg']});

for ii = 1 : figrows
    for jj = 1 : figcols
        
        % Current joint/plot
        curr = jj + (ii-1)*figcols;
        
        % Create subplot grid in row major order
        subplot(figrows, figcols, curr)
        hold on;
        
        % Plot joint profile
        plot(Time, q1(curr,:), 'DisplayName', ['q_{' Joints{curr} '}-1']);
        plot(Time, q2(curr, :), 'DisplayName', ['q_{' Joints{curr} '}-2']);
        
        % Labels
        xlabel('Time [s]');
        ylabel([Joints{curr} ' angle [rad]']);
        title({[Joints{curr} ' joint trajectory']; ['RMSE = ', num2str(rad2deg(rmse(q1(curr, :), q2(curr, :))), '%.4f'), 'rad']});
        legend;
    end
end