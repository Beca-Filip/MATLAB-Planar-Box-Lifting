tLiftOff = casadi.SX.sym('tLiftOff', 1, 1);
tDropOff = casadi.SX.sym('tDropOff', 1, 1);

t = 1 : 20;

ew_exist = (t >= tLiftOff) & (t <= tDropOff);

wF = casadi.SX.sym('wF', 3, 20);


for ii = 1 : size(wF, 2)
    wF(:, ii) = wF(:, ii) * ew_exist(ii);
end
%%
x = casadi.MX.sym('x', 6, 20);

size(jacobian(pinv(x), x))


%% 

A = casadi.SX.sym('A',3,2);
x = casadi.SX.sym('x',2);
jacobian(A*x,x)

gradient(dot(A, A), A)

[H, g] = hessian(dot(x, x), x);
disp(H)
disp(g)