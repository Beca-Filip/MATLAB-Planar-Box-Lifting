function [p,varargout] = polyCoefToTrajectory(coef, time)
%POLYCOEFTOTRAJECTORY gives back the numerical representation of the
%polynomial with given coefs, for a given vector of time. The coefficients
%must be given in ascending order (from lowest order coefficient to
%highest).
%
%   p = POLYCOEFTOTRAJECTORY(coef, t) gives back the numerical
%   representation of the polynomial.
%   [p, dp] = POLYCOEFTOTRAJECTORY(coef, t) gives back the numerical
%   representation of the polynomial and its derivative.
%   [p, dp, ddp] = POLYCOEFTOTRAJECTORY(coef, t) gives back the numerical
%   representation of the polynomial, and its first and second derivatives.
%   [p, dp, d2p, ..., dnp] = POLYCOEFTOTRAJECTORY(coef, t) gives back the
%   numerical representation of the polynomial and it's derivatives up to 
%   the n-th, as long as the order of the derivative is less than or equal 
%   to the order of the polynomial (number of coefficients).

%   Author: Filip Becanovic
%   Last Modified: 27.03.2020.
% ================= Input verification and error checks ================= %

% Coef is a column vector
coef = coef(:);

% Time is row vector
time = time(:)';

% Check the order of the polynomial
n = length(coef) - 1;

% Check if the polynomial's highest coef is not 0
if coef(n+1) == 0
    error('polyCoefToTrajectory: polynomial highest order coefficient is 0');
end

% Number of requested outputs compared to number of coefficients
if nargout-1 > n
    error('polyCoefToTrajectory: too many outputs requested');
end

% Check the number of time samples
num_samples = length(time);

% ======================================================================= %

% ======================== Coded functionalities ======================== %


% Prealocate time power matrix
t = nan(n+1, num_samples);

% Fill time power matrix
for i = 1 : n + 1
    t(i, :) = time.^(i-1);
end

% Calculate the position
p = coef' * t;

% Calculate eventual outputs
if nargout > 1
   
% For each additionnal output / derivative
for r = 1 : nargout-1
    
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
    varargout{r} = c' * t(1:n-r+1, :);
end

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