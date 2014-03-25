function [Phi, Phid] = angularvel2rpydotMatrix(rpy, rpyd)

% computes the matrix that transforms the angular velocity vector to the
% time derivatives of rpy (rolldot, pitchdot, yawdot).
% See eq. (5.41) in Craig05. Derivation in rpydot2angularvel.

p = rpy(2);
y = rpy(3);

sy = sin(y);
cy = cos(y);
sp = sin(p);
cp = cos(p);
tp = sp / cp;

% warning: note the singularities!
Phi = ...
  [cy/cp, sy/cp, 0; ...
  -sy,       cy, 0; ...
   cy*tp, tp*sy, 1];

if nargout > 1
  if nargin < 2
    error('need to pass in rpyd to compute Phid')
  end
  
  pd = rpyd(2);
  yd = rpyd(3);
  cp2 = cp^2;
  tp2 = tp^2;
  Phid = ...
    [(pd*cy*sp)/cp2 - (yd*sy)/cp, (yd*cy)/cp + (pd*sy*sp)/cp2, 0;
    -yd*cy,                                            -yd*sy, 0;
     pd*cy*(tp2 + 1) - yd*sy*tp,   pd*sy*(tp2 + 1) + yd*cy*tp, 0];
end
end