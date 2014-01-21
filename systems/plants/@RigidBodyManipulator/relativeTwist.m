function twist = relativeTwist(obj, kinsol, base, endEffector, expressedIn)
% RELATIVETWIST Computes the relative twist between base and endEffector
% @param kinsol solution structure obtained from doKinematics
% @param base index of rigid body that will be considered the base
% @param endEffector index of rigid body that will be considered the end
% effector
% @param expressedIn index of rigid body in whose frame the end result will
% be expressed
% @retval relative twist of endEffector with respect to base, expressed in
% expressedIn

% note: need obj for mex pointer info in the future

baseTwist = kinsol.twist{base};
endEffectorTwist = kinsol.twist{endEffector};

twist = ...
  transformTwists(kinsol.T{expressedIn} \ kinsol.T{endEffector}, endEffectorTwist) - ...
  transformTwists(kinsol.T{expressedIn} \ kinsol.T{base}, baseTwist);

end