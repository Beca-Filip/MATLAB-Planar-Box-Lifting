function [p, varargout] = polyCoefToTrajectory4(coef, time, order)
%POLYCOEFTOTRAJECTORY4 gives back the numerical representation of a
%polynomial with given coefficients, and its derivatives up until a given 
%order and for a given vector of time. The coefficients must be given in 
%ascending order (from lowest order coefficient to highest).
%
%   p = POLYCOEFTOTRAJECTORY4(coef, t, order=0) gives back the numerical
%   representation of the polynomial with given coefficients, by
%   calculating the values of the polynomial at times given by vector t and
%   returns the vector of calculated values.
%
%   [p, derivatives] = POLYCOEFTOTRAJECTORY4(coef, t, order) gives back the
%   numerical representation of the polynomial with given coefficients, by
%   calculating the values of the polynomial at times given by vector t. It
%   also calculates the values of the polynomial derivatives at time vector
%   t, up to a given order, and stacks them row-by-row in a matrix.
%
%   Note: Compatible with CASADI variables.

%   Author: Filip Becanovic
%   Last Modified: 03.04.2020.
% ================= Input verification and error checks ================= %

% Coef is a column vector
coef = coef(:);

% Time is row vector
time = time(:)';

% Check the order of the polynomial
n = length(coef) - 1;


% If order is negative
if order < 0
    order = 0;
end

% Check the number of time samples
num_samples = length(time);

% ======================================================================= %

% ======================== Coded functionalities ======================== %

% Prealocate derivative output matrix
if nargout > 1
    derivatives = [];
end

% Prealocate time power matrix
t = [];

% Fill time power matrix
for i = 1 : n + 1
    t = [t; time.^(i-1)];
end

% Calculate the position
p = coef' * t;

% Calculate eventual outputs
if nargout > 1
   
% For each additionnal output / derivative
for r = 1 : order
    
    % Separate coefs used for this derivative
    c = coef(r+1:end);
    
    % For each power of time calculate the multiplicator needed depending
    % on the order of the derivative
    mul = nan(n - r + 1, 1);
    for i = r : n
        mul(i - r + 1) = cutoffFactorial(i, r);
    end
    % Multiply coefs with multiplicator
    c = c .* mul;
    
    % Calculate output
    derivatives = [derivatives; c' * t(1:n-r+1, :)];
end

% Assign derivatives matrix to output
varargout{1} = derivatives;

end

end

function res = cutoffFactorial(n, k)
%CUTOFFFACTORIAL of (n, k) where 1 <= k <= n is equal to the product given
%by n * (n - 1) * ... * (n - k + 1).
    res = 1;
    for i = 0 : k - 1
        res = res * (n - i);
    end
end