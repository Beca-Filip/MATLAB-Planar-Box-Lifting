figure;
hold on;
plot(Time, COP_human(1, :), 'DisplayName', 'X-axis');
plot(Time, COP_human(2, :), 'DisplayName', 'Y-axis');
plot(Time, COP_human(3, :), 'DisplayName', 'Z-axis');
legend;
title('Human COP with Inverse Dynamics');
ylabel('Distance [m]');
xlabel('Time [s]');

figure;
hold on;
plot(Time, Forceplate.COP(:, 1), 'DisplayName', 'X-axis');
plot(Time, Forceplate.COP(:, 2), 'DisplayName', 'Y-axis');
plot(Time, Forceplate.COP(:, 3), 'DisplayName', 'Z-axis');
legend;
title('Human COP with Forceplate');
ylabel('Distance [m]');
xlabel('Time [s]');

rmse = @(a, b) sqrt(sum((a - b).^2, 'all') ./ numel(a));

figure;
hold on;
plot(Time, COP_human(1, :), 'DisplayName', 'ID');
plot(Time, Forceplate.COP(:, 1), 'DisplayName', 'FP');
ylabel('Distance [m]');
xlabel('Time [s]');
title({'Comparison of COP position between'; 'Inverse dynamics and Forceplate measurement'; ['RMSE = ' num2str(rmse(COP_human(1, :).', Forceplate.COP(:, 1)), '%.4f')]});
grid;