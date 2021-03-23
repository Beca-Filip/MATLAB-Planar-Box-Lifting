clear all;
close all;
clc;

%%  Define function and gradient
syms x1 x2 real
syms c1 c2 positive
syms f(x1, x2, c1, c2)

% Define function as a linear combination
f = [c1 c2] * [(x1 - 1)^2 + x2^2; (x1 + 1)^2 + x2^2];

% Calculate jacobian
df = jacobian(f, [x1; x2]);

%% Perform DOC

% Set some weights
c1_hat = 0.3;
c2_hat = 0.7;

fprintf("c1 = %.2f and c2 = %.2f \n\t====> f(x) = ", c1_hat, c2_hat);
disp(f);

% Get the function and gradient for these values
f_hat = subs(f, [c1 c2], [c1_hat, c2_hat]);
df_hat = subs(df, [c1, c2], [c1_hat, c2_hat]);

fprintf("\t====> f(x) = ")
disp(f_hat);

% Make these functions computable
J = matlabFunction(f_hat, 'Vars', [x1; x2]);

% Solve the DOC
x0 = randn(2, 1) * 0.1;
options = optimoptions('fminunc', 'Display', 'Off');
[x_star, f_star] = fminunc(@(x)J(x(1), x(2)), x0, options);

fprintf("\t====> x1* = %.2f and x2* = %.2f.\n", x_star(1), x_star(2));

%%

% Substitute "measured" minimum
x1_star = x_star(1);
x2_star = x_star(2);

df1 = subs(df, x1, x1_star);    % Substitute x1 first
df2 = subs(df1, x2, x2_star);   % Substitute x2 second

% Add normalization equation
df3 = [df2, c1 + c2];  

% Symbolically equate gradient to zero and normalization to 1
dfeq = (df3 == [0, 0, 1]);

% Create variable vector
c = [c1 c2];

% Solve
[c1_star, c2_star] = solve(dfeq, c); %% No solution

%% Perform IOC

% Find column 1 by substituting [1; 0]
colA1 = subs(subs(df3, c1, 1), c2, 0)';

% Find column 2 by substituting [0; 1]
colA2 = subs(subs(df3, c1, 0), c2, 1)';

% Get numerical value of matrix
A = eval([colA1, colA2]);

% Get the r.h.s
b = [0; 0; 1];

% Solve for c_star by least squares
c_star = pinv(A) * b;