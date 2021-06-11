function f = splineCoefToTrajectory(knots, coefs, x, order)
%SPLINECOEFTOTRAJECTORY takes in the knots and the coefficients values for
%a spline interpolated function and calculates its values for the given
%input vector.
%
%   y = SPLINECOEFTOTRAJECTORY(knots, coefs, x, order) takes in the knots
%   for which the spline interpolation was calculated, the coefficients of 
%   the splines and the vector of inputs x, for which the function value is
%   to be calculated. Also takes in the order of the highest derivative of
%   the function to be output.
%   Returns the function and its derivatives stacked row-by-row in a matrix
%   of size:
%   (Order + 1) x Number of samples of x
%   

% ================= Input verification and error checks ================= %

% Number of knots
N = length(knots);

% Number of trajectories
n = N - 1;

% Number of coefficients
num_coef = length(coefs);

% We must have at least 2 knots
if N <= 1
    error('splineCoefToTrajectory: Too few knots.');
end

% We must have a congruent number of polynomials and trajectories
if mod(num_coef, n) ~= 0
    error('splineCoefToTrajectory: Inconsistent number of knots and coefficients.');
end

% Determine the number of coefficients per polynomial
p = num_coef / n;

% If the number of requested outputs is bigger than degree of the
% polynomials
if nargout - 1 > p
    error('splineCoefToTrajectory: The requested number of outputs may be less or equal to the degree of the interpolated polynomials.')
end
% ======================================================================= %

% ======================== Coded functionalities ======================== %

% Number of samples
m = length(x);

% NUmber of splines
n = length(knots) - 1;

% Create buffers
% Previous remembered position inside input vector
prev_i = 1;
% Current position inside input vector
i = 1;
% Current position inside knots vector
j = 2;

% Initialize output
f = [];

% For all knots before last knot
for jj = 2:n+1
    % All timesamples less than current knot
    [q, q_der] = polyCoefToTrajectory4(coefs((jj-2)*p+1:(jj-1)*p), x(x >= knots(jj-1) & x < knots(jj)), order);
    % Add to output
    f = [f, [q; q_der]];
end

% For all knots after last knot
% All timesamples less than current knot
[q, q_der] = polyCoefToTrajectory4(coefs((n-1)*p+1:n*p), x(x >= knots(n+1)), order);
% Add to output
f = [f, [q; q_der]];

% % Until all knots or input samples have been processed
% while j <= n && i <= m
%     disp(i)
%     % If current sample is at a time lesser than the current knot, go to
%     % the next sample
%     if (x(i) <= knots(j))
%         i = i + 1;
%     % As soon as it passes the current knot, calculate the values of the
%     % samples inbetween the previous remembered position inside the input
%     % vector and the current position
%     elseif (i - prev_i > 0)
%         % Calculate trajectories and derivative values
%         [q, der_q] = polyCoefToTrajectory4(coefs(p * (j-2) + 1 : p * (j-1)), x(prev_i : i - 1), order);
%         % Store it in output structure
%         f = [f, [q; der_q]];
%         % Update previous remembered position to the current value
%         prev_i = i;
%         % Update current knot position
%         j = j + 1;
%     % If there are no samples in between then just update the knot value
%     else        
%         j = j + 1;
%     end
% end
% 
% 
% % If all samples have been processed return
% if i == m + 1
%     return
% % Else if all knots have been processed (except the last one) and not all
% % samples have been processed, process the rest of the samples according to
% % last polynomial coefficients.
% elseif j == n + 1
%     % Calculate values and derivatives for rest of the samples
%     [q, der_q] = polyCoefToTrajectory4(coefs(p * (j-2) + 1 : p * (j-1)), x(prev_i : m), order);
%     % Store them in output structure
%     f = [f, [q; der_q]];
% end

end

