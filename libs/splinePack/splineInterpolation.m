function coefficients = splineInterpolation(x, y, p, boundaryConditions)
%SPLINEINTERPOLATION approximates a function using a piece-wise polynomial
%representation and gives back the coefficients of said polynomials.
%   coefficients = SPLINEINTERPOLATION(x, y, p, boundaryConditions)
%   gives back a coefficients vector of polynomials of a given order p
%   approximating the function f(x) = y given by the vector of knot points 
%   x and the corresponding function values y. A representation of order p 
%   requires another p-1 boundary conditions for the coefficients to be 
%   uniquely defined. In this implementation boundary conditions must be 
%   passed. 
%   The variable boundaryCondition is an array variable of size (p-1)x3 
%   where each row represents one equality condition upon the spline 
%   representation of the function, or one of its first p derivatives. 
%   Each row is represented in the following format [der, x_val, y_val]:
%   -der: Order of the derivative upon which the condition is placed, it
%   can range from 0 to p.
%   -x_val: the value of x at which the condition is placed
%   -y_val: the value which the derivative of order der should take at
%   x_val

%   Author: Filip Becanovic
%   Last Modified: 31.03.2020.
% ================= Input verification and error checks ================= %

% Number of polynomials
n = length(x) - 1;

% Number of boundary conditions
[c, three] = size(boundaryConditions);

% Check that dimensions are OK
if all([((three ~= 3) || (c ~= p - 1)) , ((three ~= p - 1) || (c ~= 3))])
    error('spline: boundaryConditions incorrect format or wrong number of conditions');
end
if three ~= 3
    boundaryConditions = boundaryConditions';
    [c, three] = size(boundaryConditions);
end

% ======================================================================= %

% ======================== Coded functionalities ======================== %

% Matrix A
A = zeros((p+1) * n, (p+1) * n);

% Matrix b
b = zeros((p+1) * n, 1);

% For each polynomial up to the last add interior and exterior knot-point 
% equalities
for i = 1 : n
    
    % Conditions upon the endpoints of the polynomials
    % For each coefficient
    for j = 1 : p+1
        
        % Equate the value of the i-th polynomial at the i-th time sample 
        % with the true value
        A((p+1) * (i-1) + 1, (p+1) * (i-1) + j) = x(i).^(j-1);
        
        % Equate the value of the i-th polynomial at the (i+1)-th time
        % sample with the true value or with the value of the (i+1)-th
        % polynomial at the same time sample
        A((p+1) * (i-1) + 2, (p+1) * (i-1) + j) = x(i+1).^(j-1);
    end
    
    % Equate the value of the i-th polynomial at the i-th time sample 
    % with the true value
    b((p+1) * (i-1) + 1) = y(i);
    % Equate the value of the i-th polynomial at the (i+1)-th time
    % sample with the true value or with the value of the (i+1)-th
    % polynomial at the same time sample
    b((p+1) * (i-1) + 2) = y(i+1);
end

% If the order of the polynomial is greater than one, add boundary
% conditions and derivative matching at interior knot points
if p > 1

% For each polynomial up to the next-to last one add interior point
% derivative equalities
for i = 1 : n - 1
    
    % For each derivative up to order-1
    for d = 1 : p - 1

        % Conditions upon the derivatives at the endpoints of the polynomial
        % For each coefficient
        for j = d+1 : p+1
        
            % Equate the value of the d-th derivative of the i-th 
            % polynomial at the (i+1)-th time sample with the value of the
            % derivative of the (i+1)-th polynomial at the same time sample
            A((p+1) * (i-1) + 2 + d, (p+1) * (i-1) + j) = cutoffFactorial(j-1, d) * x(i+1).^(j-1-d);
            A((p+1) * (i-1) + 2 + d, (p+1) * (i-1) + p + 1 + j) = -cutoffFactorial(j-1, d) * x(i+1).^(j-1-d);
        end

    end

end

% For each boundary condition incorporate it into equations
for num_con = 1:c
    
    % Order of the derivative upon which condition is placed
    d = boundaryConditions(num_con, 1);
    if (d < 0) || (d > p)
        error("spline: boundaryCondition the derivative order can't be less than 0 or bigger than polynomial order.");
    end
    
    % Value specified for the derivative
    y_c = boundaryConditions(num_con, 3);
    
    % Independent variable value at which the condition is placed
    x_c = boundaryConditions(num_con, 2);
    
    % Locate which polynomial is concerned by the condition by looking
    % for the closest knot-point that's bigger than it
    num_pol = find(x > x_c, 1, 'first');
    % If none is found, that means it is bigger than all and that the last
    % polynomial is concerned
    if isempty(num_pol)
        num_pol = n;
    % If i-th knot is found, then it concerns the (i-1)-th polynomial,
    % except if i is one, then it is also the first polynomial.
    elseif num_pol > 1
        num_pol = num_pol-1;
    end
    
    % For each coefficient dependent upon the order of the derivative
    for j = d+1 : p+1
        % Equate the d-th order derivative of num_pol-th polynomial as the 
        % num_con-th condition at point x_c with the value given by y_c.
        A((p+1) * (n-1) + 2 + num_con, (p+1) * (num_pol-1) + j) = cutoffFactorial(j-1, d) * x_c.^(j-1-d);
    end
    
    % Equate the d-th order derivative of num_pol-th polynomial as the 
    % num_con-th condition at point x_c with the value given by y_c.
    b((p+1) * (n-1) + 2 + num_con) = y_c;
end

end

coefficients = A^(-1) * b;

end


function res = cutoffFactorial(n, k)
%CUTOFFFACTORIAL of (n, k) where 1 <= k <= n is equal to the product given
%by n * (n - 1) * ... * (n - k + 1).
    res = 1;
    for i = 0 : k - 1
        res = res * (n - i);
    end
end