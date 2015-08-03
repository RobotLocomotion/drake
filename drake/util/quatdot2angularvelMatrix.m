function [M, dM] = quatdot2angularvelMatrix(q)
% Computes matrix that maps quaternion derivative to angular velocity in
% base frame
%
% from Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf
%
% @param q a quaternion
%
% @retval M matrix such that omega = M * qd, where omega is the angular
% velocity in base frame and qd is the time derivative of q
% @retval dM gradient of M with respect to q. Quaternion normalization is
% included in the gradient.

compute_gradient = nargout > 1;

if compute_gradient
  [qtilde, dqtildedq] = normalizeVec(q);
else
  qtilde = normalizeVec(q);
end

q0 = qtilde(1);
qv = qtilde(2 : 4);

M = 2 * [-qv, q0 * eye(3) + vectorToSkewSymmetric(qv)];

if compute_gradient
  dMdqtilde =      [...
    0    -2     0     0;
    0     0    -2     0;
    0     0     0    -2;
    2     0     0     0;
    0     0     0     2;
    0     0    -2     0;
    0     0     0    -2;
    2     0     0     0;
    0     2     0     0;
    0     0     2     0;
    0    -2     0     0;
    2     0     0     0];
  dM = dMdqtilde * dqtildedq;
end

end