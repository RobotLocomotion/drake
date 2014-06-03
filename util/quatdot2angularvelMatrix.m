function [M, dM] = quatdot2angularvelMatrix(q)
% Computes matrix that maps quaternion derivative to angular velocity in
% base frame
%
% from Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf

q0 = q(1);
qv = q(2 : 4);

M = 2 * [-qv, q0 * eye(3) + vectorToSkewSymmetric(qv)];

compute_gradient = nargout > 1;
if compute_gradient
  dM =      [...
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
end

end