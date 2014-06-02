function [spatial_accel, dspatial_accel] = transformSpatialAcceleration(spatial_accel, T, twists, base, body, old_frame, new_frame, dspatial_accel, dTdq, dtwistsdq)
% TRANSFORMSPATIALACCELERATION Transforms a spatial acceleration vector
% (derivative of a twist) to a different reference frame. 
% The formula for changing the frame in which a spatial acceleration vector
% is expressed is derived by differentiating the transformation formula for
% twists.
% @param spatialAccel a spatial acceleration vector
% @param transforms homogeneous transforms from link to world (usually
% obtained from doKinematics as kinsol.T)
% @param twists twists of links with respect to world
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

compute_gradient = nargout > 1;
if compute_gradient
  if nargin < 10
    error('need dTdq and dtwistsdq to compute gradient')
  end
end

if compute_gradient
  [twist_of_body_wrt_base, dtwist_of_body_wrt_base] = relativeTwist(T, twists, base, body, old_frame, dTdq, dtwistsdq);
  [twist_of_old_wrt_new, dtwist_of_old_wrt_new] = relativeTwist(T, twists, new_frame, old_frame, old_frame, dTdq, dtwistsdq);
else
  twist_of_body_wrt_base = relativeTwist(T, twists, base, body, old_frame);
  twist_of_old_wrt_new = relativeTwist(T, twists, new_frame, old_frame, old_frame);
end

T_old = T{old_frame};
T_new = T{new_frame};
T_new_inv = homogTransInv(T_new);
T_old_to_new = T_new_inv * T_old;
adT_old_wrt_new = twistAdjoint(twist_of_old_wrt_new);

spatial_accel_temp = adT_old_wrt_new * twist_of_body_wrt_base + spatial_accel;
spatial_accel = transformAdjoint(T_old_to_new) * spatial_accel_temp;

% this could be faster:
% omegaOfBodyWrtBase = twistOfBodyWrtBase(1 : 3);
% vOfBodyWrtBase = twistOfBodyWrtBase(4 : 6);
%
% omegaOfOldWrtNew = twistOfOldWrtNew(1 : 3);
% vOfOldWrtNew = twistOfOldWrtNew(4 : 6);
%
% omegadot = spatialAccel(1 : 3);
% vdot = spatialAccel(4 : 6);
%
% ROldToNew = transformFromOldToNew(1 : 3, 1 : 3);
% pOldToNew = transformFromOldToNew(1 : 3, 4);
%
% omegadotNew = ROldToNew * (cross(omegaOfOldWrtNew, omegaOfBodyWrtBase) + omegadot);
% vdotNew = cross(pOldToNew, omegadotNew) + ROldToNew * (cross(vOfOldWrtNew, omegaOfBodyWrtBase)...
%   + cross(omegaOfOldWrtNew, vOfBodyWrtBase) + vdot);
%
% spatialAccel = [omegadotNew; vdotNew];

if compute_gradient
  dT_old = dTdq{old_frame};
  dT_new = dTdq{new_frame};
  dT_old_to_new = matGradMultMat(T_new_inv, T_old, dinvT(T_new, dT_new), dT_old);
  dadT_old_to_new = dtwistAdjoint(dtwist_of_old_wrt_new);
  
  dspatial_accel_temp = matGradMultMat(adT_old_wrt_new, twist_of_body_wrt_base, dadT_old_to_new, dtwist_of_body_wrt_base) + dspatial_accel;
  dspatial_accel = dAdHTimesX(T_old_to_new, spatial_accel_temp, dT_old_to_new, dspatial_accel_temp);
end
end