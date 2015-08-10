function  [Xj,S] = jcalc( pitch, q )

% jcalc  Calculate joint transform and motion subspace.
% [Xj,S]=jcalc(pitch,q) calculates the joint transform and motion subspace
% matrices for a revolute (pitch==0), prismatic (pitch==inf) or helical
% (pitch==any other value) joint.  For revolute and helical joints, q is
% the joint angle.  For prismatic joints, q is the linear displacement.

if pitch == 0				% revolute joint
  Xj = Xrotz(q);
  S = [0;0;1;0;0;0];
elseif pitch == inf			% prismatic joint
  Xj = Xtrans([0 0 q]);
  S = [0;0;0;0;0;1];
else					% helical joint
  Xj = Xrotz(q) * Xtrans([0 0 q*pitch]);
  S = [0;0;1;0;0;pitch];
end
