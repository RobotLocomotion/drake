function ret = quatdot2angularvelMatrix(q)
% Computes matrix that maps quaternion derivative to angular velocity in
% base frame
% 
% from Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf

q0 = q(1);
qv = q(2 : 4);

ret = 2 * [-qv, q0 * eye(3) + vectorToSkewSymmetric(qv)];
end