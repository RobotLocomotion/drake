function spatialAccel = transformSpatialAcceleration(obj, spatialAccel, kinsol, base, body, oldExpressedIn, newExpressedIn)
% TRANSFORMSPATIALACCELERATION Transforms a spatial acceleration vector
% (derivative of a twist) to a different reference frame. 
% The formula for changing the frame in which a spatial acceleration vector
% is expressed is derived by differentiating the transformation formula for
% twists.
% @param spatialAccel a spatial acceleration vector
% @param kinsol solution structure obtained from doKinematics
% @param base index of body with respect to which spatialAccel expresses
% acceleration
% @param body index of body of which spatialAccel expresses the
% acceleration
% @param oldExpressedIn index of body in whose reference frame spatialAccel
% is initially expressed
% @param newExpressedIn index of body in whose reference frame the final
% result should be expressed
% @retval spatialAccel, transformed from oldExpressedIn to newExpressedIn
% frame

twistOfBodyWrtBase = obj.relativeTwist(kinsol, base, body, oldExpressedIn);
twistOfOldWrtNew = obj.relativeTwist(kinsol, newExpressedIn, oldExpressedIn, oldExpressedIn);
transformFromOldToNew = kinsol.T{newExpressedIn} \ kinsol.T{oldExpressedIn};

% spatialAccel = transformAdjoint(transformFromOldToNew) * (twistAdjoint(twistOfOldWrtNew) * twistOfBodyWrtBase + spatialAccel);
% function adT = twistAdjoint(twist)
% omega = twist(1 : 3);
% v = twist(4 : 6);
% adT = [vectorToSkewSymmetric(omega), zeros(3, 3);
%        vectorToSkewSymmetric(v), vectorToSkewSymmetric(omega)];
% end
%
% this should be faster:
omegaOfBodyWrtBase = twistOfBodyWrtBase(1 : 3);
vOfBodyWrtBase = twistOfBodyWrtBase(4 : 6);

omegaOfOldWrtNew = twistOfOldWrtNew(1 : 3);
vOfOldWrtNew = twistOfOldWrtNew(4 : 6);

omegadot = spatialAccel(1 : 3);
vdot = spatialAccel(4 : 6);

ROldToNew = transformFromOldToNew(1 : 3, 1 : 3);
pOldToNew = transformFromOldToNew(1 : 3, 4);

omegadotNew = ROldToNew * (cross(omegaOfOldWrtNew, omegaOfBodyWrtBase) + omegadot);
vdotNew = cross(pOldToNew, omegadotNew) + ROldToNew * (cross(vOfOldWrtNew, omegaOfBodyWrtBase)...
  + cross(omegaOfOldWrtNew, vOfBodyWrtBase) + vdot);

spatialAccel = [omegadotNew; vdotNew];

end