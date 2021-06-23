%% Run the Motion_Optimization_Human script before and leave the workspace.

J = FKM_Jacobian_nDOF_Sample(q(:, 1), L);
dq = diff(q(:, 1:2), 1, 2)/Ts;

v_d = J * dq;

F = FKM_nDOF_Tensor(q(:, 1:2), L);
v_n = diff(F(1:3, 4, end, :), 1, 4)/Ts;

fprintf('Velocity\nAnalytically:\t\t\t\tNumerically:\n[%.4f,%.4f]\t\t\t\t[%.4f,%.4f]\n', v_d, v_n(1:2));

%%

dJ = FKM_dJacobian_nDOF_Sample(q(:, 1), dq, L);
ddq = diff(q(:, 1:3), 2, 2)/Ts^2;

a_d = dJ * dq + J * ddq;

F2 = FKM_nDOF_Tensor(q(:, 1:3), L);
a_n = diff(F2(1:3, 4, end, :), 2, 4)/Ts^2;


fprintf('Acceleration\nAnalytically:\t\t\t\tNumerically:\n[%.4f,%.4f]\t\t\t\t[%.4f,%.4f]\n', a_d, a_n(1:2));

%% Whole traj

dq = diff(q, 1, 2)/Ts;

v_d = FKM_Velocity_nDOF_Tensor(q(:, 1:end-1), dq, L);

F = FKM_nDOF_Tensor(q, L);

v_n = squeeze(diff(F(1:2, 4, end, :), 1, 4)) / Ts;

figure;
subplot(2, 1, 1)
hold on;
plot(Time(1:end-1), v_d(1, :), 'DisplayName', 'Analytical');
plot(Time(1:end-1), v_n(1, :), '--', 'DisplayName', 'Numerical');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('End-Effector velocity in the X direction');
grid;
legend;
subplot(2, 1, 2)
hold on;
plot(Time(1:end-1), v_d(2, :), 'DisplayName', 'Analytical');
plot(Time(1:end-1), v_n(2, :), '--', 'DisplayName', 'Numerical');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('End-Effector velocity in the Y direction');
grid;
legend;


ddq = diff(q, 2, 2)/Ts^2;

a_d = FKM_Acceleration_nDOF_Tensor(q(:, 1:end-2), dq(:, 1:end-1), ddq, L);

a_n = squeeze(diff(F(1:2, 4, end, :), 2, 4)) / Ts^2;

figure;
subplot(2, 1, 1)
hold on;
plot(Time(1:end-2), a_d(1, :), 'DisplayName', 'Analytical');
plot(Time(1:end-2), a_n(1, :), '--', 'DisplayName', 'Numerical');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('End-Effector acceleration in the X direction');
grid;
legend;
subplot(2, 1, 2)
hold on;
plot(Time(1:end-2), a_d(2, :), 'DisplayName', 'Analytical');
plot(Time(1:end-2), a_n(2, :), '--', 'DisplayName', 'Numerical');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('End-Effector accelration in the Y direction');
grid;
legend;

%%

COM = COM_nDOF_Cell(q,L,M,CMP);
COM = cell2mat(COM);

v_COM_n = diff(COM, 1, 2) / Ts;

v_COM_d = COM_Velocity_nDOF_Tensor(q(:, 1:end-1), dq, L, M, CMP);

figure;
subplot(2,1,1)
hold on;
plot(Time(1:end-1), v_COM_n(1, :), 'DisplayName', 'Numeric');
plot(Time(1:end-1), v_COM_d(1, :), 'DisplayName', 'Analytic');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Center of Mass velocity in the X direction');
grid;
legend;
subplot(2,1,2)
hold on;
plot(Time(1:end-1), v_COM_n(2, :), 'DisplayName', 'Numeric');
plot(Time(1:end-1), v_COM_d(2, :), 'DisplayName', 'Analytic');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Center of Mass velocity in the Y direction');
grid;
legend;

%%

a_COM_n = diff(COM, 2, 2) / Ts^2;

a_COM_d = COM_Acceleration_nDOF_Tensor(q(:, 1:end-2), dq(:, 1:end-1), ddq, L, M, CMP);

figure;
subplot(2,1,1)
hold on;
plot(Time(1:end-2), a_COM_n(1, :), 'DisplayName', 'Numeric');
plot(Time(1:end-2), a_COM_d(1, :), 'DisplayName', 'Analytic');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Center of Mass Acceleration in the X direction');
grid;
legend;
subplot(2,1,2)
hold on;
plot(Time(1:end-2), a_COM_n(2, :), 'DisplayName', 'Numeric');
plot(Time(1:end-2), a_COM_d(2, :), 'DisplayName', 'Analytic');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Center of Mass Acceleration in the Y direction');
grid;
legend;