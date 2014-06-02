function [twist, dtwist] = relativeTwist(transforms, twists, base, end_effector, expressed_in, dtransforms, dtwists)
% RELATIVETWIST Computes the relative twist between base and endEffector
% @param transforms homogeneous transforms from link to world (usually
% obtained from doKinematics as kinsol.T)
% @param twists twists of links with respect to world, expressed in world
% @param base index of rigid body that will be considered the base
% @param endEffector index of rigid body that will be considered the end
% effector
% @param expressedIn index of rigid body in whose frame the end result will
% be expressed
% @retval relative twist of endEffector with respect to base, expressed in
% expressedIn

compute_gradient = nargout > 1;

if compute_gradient
  if nargin < 7
    error('need dtransforms and dtwists to compute gradient');
  end
end

twist = twists{end_effector} - twists{base};
if compute_gradient
  dtwist = dtwists{end_effector} - dtwists{base};
end

if expressed_in ~= 1
  T = transforms{expressed_in};
  Tinv = homogTransInv(T);
  if compute_gradient
    dT = dtransforms{expressed_in};
    dTinv = dinvT(T, dT);
    dtwist = dAdHTimesX(Tinv, twist, dTinv, dtwist);
  end
  twist = transformTwists(Tinv, twist);
end
end