function ret = quadFormGrad(X, P, dX)
% computes the gradient of X(q)' * P * X(q) with respect to a vector q, given
% X, P, and dX/dq
% see http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/calculus.html#deriv_quad

% TODO: make more efficient
n = size(X, 2);
ret = kron(eye(n), X' * P) * dX + kron(X' * P', eye(n)) * transposeGrad(dX, size(X));

end