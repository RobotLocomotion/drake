function [M, dM] = angularvel2quatdotMatrix(q)
% Computes matrix that maps angular velocity in world-fixed frame to
% quaternion derivative
%
% From Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf
% @param q is a quaternion

compute_gradient = nargout > 1;
% NOTE: enabling normalization causes testForwardKinV to fail.

% if compute_gradient
%   [qtilde, dqtildedq] = normalizeVec(q);
% else
%   qtilde = normalizeVec(q);
% end
qtilde = q;

qs = qtilde(1);
qv = qtilde(2 : 4);

M = 1/2 * ...
  [-qv';
  qs * eye(3) - vectorToSkewSymmetric(qv)];

if compute_gradient
  dM = [...
       0      -0.5   0      0;
       0.5    0      0      0;
       0      0      0      -0.5;
       0      0      0.5    0;
       0      0      -0.5   0;
       0      0      0      0.5;
       0.5    0      0      0;
       0      -0.5   0      0;
       0      0      0      -0.5;
       0      0      -0.5   0;
       0      0.5    0      0;
       0.5    0      0      0]; % * dqtildedq;
end
end