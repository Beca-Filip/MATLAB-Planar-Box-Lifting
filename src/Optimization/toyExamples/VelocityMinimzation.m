clear all;
close all;
clc;

%% Include libs and data
% Add CASADI library
if ismac
    % Code to run on Mac platform
    addpath('../../../libs/casadi-osx-matlabR2015a-v3.5.1')
elseif isunix
    % Code to run on Linux platform
    addpath('../../../libs/casadi-linux-matlabR2015a-v3.5.1')     % Add CASADI to path
elseif ispc
    % Code to run on Windows platform
    addpath('../../../libs/casadi-windows-matlabR2016a-v3.5.5')
else
    disp('Platform not supported')
end

% Add spline library
addpath("../../../libs/splinePack"); 
% Add charlotte's data
addpath("../../../data/3DOF/Charlotte")
% Load the parameters
load Charlotte.mat
load param_variable.mat

% Extract a piece of the trajectories
TimeSpan=4138:4300;
q=q(:,TimeSpan); qh = q;
dq=dq(:, TimeSpan); dqh = dq;
ddq=ddq(:, TimeSpan); ddqh = ddq;
dddq=dddq(:, TimeSpan); dddqh = dddq;
clear q dq ddq dddq

%% Define time related parameters
% Number of points
M = 163;

% Start time
t0 = 0;

% Final time
tf = 1.62*4;

% Time vector
t = linspace(t0, tf, M);

% Sampling rate
Ts = (tf - t0) / (M-1);

%% Spline interpolation parameters

% Number of knots
N = 11;

% Spline Knot Indices
ski = floor(linspace(1, M, N));

% Spline Knot Points - abscissa - 1st joint
skp = t(ski);

% Order of interpolation
p = 5;

% Velocity and acceleration boundary conditions
boundaries = [
              1, t(1), 0;       % Zero velocity at first time
              2, t(1), 0;       % Zero acceleration at first time
              1, t(end), 0;     % Zero velocity at the last time
              2, t(end), 0;     % Zero acceleration at the last time
];

%% Optimization parameters

% Number of variables: Number of trajectories times number of knots per
% traj
NumVars = size(qh, 1) * N;

% Set RNG for deterministic behavior
% rng(36);

% Generate a specific quadratic matrix using casadi
    % Generate a casadi variable containing control points from all
    % trajectories
    x_cas = casadi.SX.sym('x_cas', 1, NumVars);
    
    % Distribute control points between trajectories
    q1_knot_cas = x_cas(1:N);
    q2_knot_cas = x_cas(N+1:2*N);
    q3_knot_cas = x_cas(2*N+1:3*N);
    
    % Interpolate each one
    cq1_cas = splineInterpolation2(skp, q1_knot_cas, p, boundaries);
    cq2_cas = splineInterpolation2(skp, q2_knot_cas, p, boundaries);
    cq3_cas = splineInterpolation2(skp, q3_knot_cas, p, boundaries);
    
    q1_cas = splineCoefToTrajectory(skp, cq1_cas, t, 1);    % Order = 1 to get velocities in 2nd row
    q2_cas = splineCoefToTrajectory(skp, cq2_cas, t, 1);    % Order = 1 to get velocities in 2nd row
    q3_cas = splineCoefToTrajectory(skp, cq3_cas, t, 1);    % Order = 1 to get velocities in 2nd row
    
    % Get sum squared of joint velocities
    J = sum(q1_cas(2, :).^2 + q2_cas(2, :).^2 + q3_cas(2, :).^2);
    fprintf("We assume that the Optimization problem has a cost function of form\n");
    fprintf("1/2 * x' * H * x + f' * x + c\n");
    
    % Gradient
    gradJ = jacobian(J, x_cas)';
    
    % Hessian
    hessJ = jacobian(gradJ, x_cas)';
    
    % Create evaluation procedures
    evalGradJ = casadi.Function('evalGradJ', {x_cas}, {gradJ});
    evalHessJ = casadi.Function('evalHessJ', {x_cas}, {hessJ});
    
    % Check if hessian is constant (it should theoretically be)
    x1_r = randn(1, NumVars);
    x2_r = randn(1, NumVars);
    
    h1_r = full(evalHessJ(x1_r));
    h2_r = full(evalHessJ(x2_r));
    
    if isequal(h1_r, h2_r)
        fprintf("The Hessian H is indeed constant, so the function must be quadratic.\n");
        H = h1_r;   % Set hessian
        H = (H + H')/2;
    end
    
    % Get the linear vector
%    	g1_r = full(evalGradJ(x1_r));
%     g2_r = full(evalGradJ(x2_r));
%     
%     f1_r = g1_r - h1_r * x1_r';
%     f2_r = g2_r - h2_r * x2_r';
%     
%     if isequal(f1_r, f2_r)
%         fprintf("The Vector term is also constant.")
%     end
    f_cas = jacobian(J, x_cas)' - H * x_cas';
    
    evalLinearTerm = casadi.Function('evalLinearTerm', {x_cas}, {f_cas});
    
    f1_r = full(evalLinearTerm(x1_r));
    f2_r = full(evalLinearTerm(x2_r));
    
    fprintf("The norm of the difference between the linear term evaluated at two random points is %.4f.\n", norm(f1_r - f2_r));
    
%%

% The linear vector is defined by previous calculations
f = zeros(NumVars, 1);

% Define initial and final positions
qi = qh(:, 1);
qf = qh(:, end);

% Allocate equality constraint matrix
Aeq = zeros(6, NumVars);
beq = zeros(6, 1);

% Set adequate elements in equality constraint matrices
% First knot of first joint constraint
Aeq(1, 1)       = 1;      
beq(1)          = qi(1);
% Last knot of first joint constraint
Aeq(2, N)       = 1;
beq(2)          = qf(1);

% First knot of second joint constraint
Aeq(3, N+1)     = 1;      
beq(3)          = qi(2);
% Last knot of first joint constraint
Aeq(4, 2*N)     = 1;
beq(4)          = qf(2);

% First knot of first joint constraint
Aeq(5, 2*N+1)   = 1;      
beq(5)          = qi(3);
% Last knot of first joint constraint
Aeq(6, 3*N)     = 1;
beq(6)          = qf(3);

% Define variable bounds
One = ones(N , 1);
lb = [param.qlim(1, 1) * One; param.qlim(1, 2) * One; param.qlim(1, 3) * One];
ub = [param.qlim(2, 1) * One; param.qlim(2, 2) * One; param.qlim(2, 3) * One];

% Perform optimization
x0 = randn(NumVars, 1) * 0.1;
options = optimoptions('quadprog', 'Display', 'Off');
[x_star, f_star, ef_star, out_star] = quadprog(H, f, [], [], Aeq, beq, lb, ub, x0, options);

%% Perform interpolation on output

% Create opts structure for animation
opts = struct();

% Define segment lengths
L = [param.L1, param.L2, param.L3];

% Define segment relative masses
M = [param.M1, param.M2, param.M3] / param.Mtot;

% Define individial segment COM position vectors and place them in a matrix
C1 = [param.MX1; param.MY1; 0] / param.M1;
C2 = [param.MX2; param.MY2; 0] / param.M2;
C3 = [param.MX3; param.MY3; 0] / param.M3;
CMP = [C1, C2, C3];

% Exctract knots for each joint
q1_knot_star = x_star(1:N)';
q2_knot_star = x_star(N+1:2*N)';
q3_knot_star = x_star(2*N+1:3*N)';

% Get spline interpolation coefs for each joint trajectory
polycoefs1_star = splineInterpolation(skp, q1_knot_star, p, boundaries);
polycoefs2_star = splineInterpolation(skp, q2_knot_star, p, boundaries);
polycoefs3_star = splineInterpolation(skp, q3_knot_star, p, boundaries);

% Get spline trajectories from coeffs
q1_star = splineCoefToTrajectory(skp, polycoefs1_star, t, 0);
q2_star = splineCoefToTrajectory(skp, polycoefs2_star, t, 0);
q3_star = splineCoefToTrajectory(skp, polycoefs3_star, t, 0);

% Pack trajectories into a single matrix
q_star = [q1_star; q2_star; q3_star];

% Pack knots into a single matrix
q_knot_star = [q1_knot_star; q2_knot_star; q3_knot_star];

% Perform FKM for the knots
T_knot_star = FKM_3DOF_Tensor(q_knot_star, L);
X_knot_star = squeeze(T_knot_star(1, 4, end, :));
Y_knot_star = squeeze(T_knot_star(2, 4, end, :));

% Perform FKM for all samples
T_star = FKM_3DOF_Tensor(q_star, L);
X_star = squeeze(T_star(1, 4, end, :));
Y_star = squeeze(T_star(2, 4, end, :));

% Perform COM FKM for the knots
COM_knot_star = COM_3DOF_Tensor(q_knot_star, L, M, CMP);

% Perform COM FKM for the whole trajectory
COM_star = COM_3DOF_Tensor(q_star, L, M, CMP);

% Create background plotting function for plotting knot points
% opts.bgrPlot = @()plot(X_knot, Y_knot, 'go', X, Y, 'DisplayName', 'KnotPoints&Traj', 'LineWidth', 1.3);
opts.bgrPlot = @()backgroundPlot(X_knot_star, Y_knot_star, X_star, Y_star, COM_knot_star, COM_star);

% Create a tool option for aesthetics
opts.tool = struct("type", "circle", "diameter", min(L)*0.25);

% Create a legend for aesthetics
opts.generateLegend = true;
opts.legendParameters = {"Location", "SouthWest"};

% Call the animate function
Animate_3DOF(q_star, L, Ts, opts);

function backgroundPlot(X_knot, Y_knot, X, Y, COM_knot, COM)
    plot(X_knot, Y_knot, 'go', X, Y, 'DisplayName', 'Head Traj', 'LineWidth', 1.3);
    plot(COM_knot(1, :), COM_knot(2, :), 'bo', COM(1, :), COM(2, :), 'DisplayName', 'CoM Traj' , 'LineWidth', 1.3);
end