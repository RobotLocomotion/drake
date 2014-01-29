function ret = angularvel2quatdotMatrix(q)
% Computes matrix that maps angular velocity in space-fixed frame to
% quaternion derivative
% 
% From Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf

qs = q(1);
qv = q(2 : 4);

ret = 1/2 * ...
  [-qv';
   qs * eye(3) - vectorToSkewSymmetric(qv)];

end