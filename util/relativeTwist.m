function twist = relativeTwist(transforms, twists, base, endEffector, expressedIn)
% RELATIVETWIST Computes the relative twist between base and endEffector
% @param transforms homogeneous transforms from link to world (usually
% obtained from doKinematics as kinsol.T)
% @param twists twists of links with respect to world
% @param base index of rigid body that will be considered the base
% @param endEffector index of rigid body that will be considered the end
% effector
% @param expressedIn index of rigid body in whose frame the end result will
% be expressed
% @retval relative twist of endEffector with respect to base, expressed in
% expressedIn

baseTwist = twists{base};
endEffectorTwist = twists{endEffector};

twist = ...
  transformTwists(transforms{expressedIn} \ transforms{endEffector}, endEffectorTwist) - ...
  transformTwists(transforms{expressedIn} \ transforms{base}, baseTwist);

end