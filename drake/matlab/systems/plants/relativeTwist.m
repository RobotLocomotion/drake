function [twist, dtwist] = relativeTwist(kinsol, base, end_effector, expressed_in)
% Computes the relative twist between base and end effector
% @param transforms homogeneous transforms from link to world (usually
% obtained from doKinematics as kinsol.T)
% @param kinsol solution structure returned by doKinematics
% @param twists twists of links with respect to world, expressed in world
% @param base index of rigid body that will be considered the base
% @param end_effector index of rigid body that will be considered the end
% effector
% @param expressed_in index of rigid body in whose frame the end result
% will be expressed
% @retval relative twist of end_effector with respect to base, expressed in
% expressed_in

compute_gradient = nargout > 1;

twist = kinsol.twists{end_effector} - kinsol.twists{base};
if compute_gradient
  dtwist = kinsol.dtwistsdq{end_effector} - kinsol.dtwistsdq{base};
end

if expressed_in ~= 1
  T = kinsol.T{expressed_in};
  Tinv = homogTransInv(T);
  if compute_gradient
    dT = kinsol.dTdq{expressed_in};
    dTinv = dHomogTransInv(T, dT);
    dtwist = dTransformSpatialMotion(Tinv, twist, dTinv, dtwist);
  end
  twist = transformAdjoint(Tinv) * twist;
end
end