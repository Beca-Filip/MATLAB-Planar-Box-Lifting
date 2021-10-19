function compare_trajectories(q1, q2, Time)
%COMPARE_TRAJECTORIES compares two trajectories.
%
%   COMPARE_TRAJECTORIES(q1, q2, Time)

% Define rmse 
rmse = @(a,b) sqrt(sum((a-b).^2 / numel(a), 'all'));

% Joint names
Joints = {'Ankle', 'Knee', 'Hip', 'Back', 'Shoulder', 'Elbow'};
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
        title({[Joints{curr} ' joint trajectory']; ['RMSE = ', num2str(rad2deg(rmse(q1(curr, :), q2(curr, :))), '%.4f'), 'deg']});
        legend;
    end
end

end