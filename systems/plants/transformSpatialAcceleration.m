function [spatial_accel, dspatial_accel] = transformSpatialAcceleration(kinsol, base, body, old_frame, new_frame, spatial_accel, dspatial_accel)
% TRANSFORMSPATIALACCELERATION Transforms a spatial acceleration vector
% (derivative of a twist) to a different reference frame. 
% The formula for changing the frame in which a spatial acceleration vector
% is expressed is derived by differentiating the transformation formula for
% twists.
% @param solution structure returned by RigidBodyManipulator/doKinematics
% @param base index of body with respect to which spatialAccel expresses
% acceleration
% @param body index of body of which spatialAccel expresses the
% acceleration
% @param old_frame index of body in whose reference frame spatialAccel
% is initially expressed
% @param new_frame index of body in whose reference frame the final
% result should be expressed
% @param spatialAccel a spatial acceleration vector
% @retval spatial_accel, transformed from oldExpressedIn to newExpressedIn
% frame
% @retval dspatial_accel, gradient of spatial_accel w.r.t. q and v

if old_frame == new_frame
  return
end

compute_gradient = nargout > 1;
if compute_gradient
  if nargin < 7
    error('need dspatial_accel to compute transformed dspatial_accel');
  end
end

if compute_gradient
  [twist_of_body_wrt_base, dtwist_of_body_wrt_base] = relativeTwist(kinsol, base, body, old_frame);
  [twist_of_old_wrt_new, dtwist_of_old_wrt_new] = relativeTwist(kinsol, new_frame, old_frame, old_frame);
else
  twist_of_body_wrt_base = relativeTwist(kinsol, base, body, old_frame);
  twist_of_old_wrt_new = relativeTwist(kinsol, new_frame, old_frame, old_frame);
end

T_old = kinsol.T{old_frame};
T_new = kinsol.T{new_frame};
T_new_inv = homogTransInv(T_new);
T_old_to_new = T_new_inv * T_old;

spatial_accel_temp = crm(twist_of_old_wrt_new) * twist_of_body_wrt_base + spatial_accel;
spatial_accel = transformAdjoint(T_old_to_new) * spatial_accel_temp;

if compute_gradient
  dT_old = kinsol.dTdq{old_frame};
  dT_new = kinsol.dTdq{new_frame};
  dT_old_to_new = matGradMultMat(T_new_inv, T_old, dHomogTransInv(T_new, dT_new), dT_old);
  
  dspatial_accel_temp = dcrm(twist_of_old_wrt_new, twist_of_body_wrt_base, dtwist_of_old_wrt_new, dtwist_of_body_wrt_base) + dspatial_accel;
  dspatial_accel = dTransformSpatialMotion(T_old_to_new, spatial_accel_temp, dT_old_to_new, dspatial_accel_temp);
end
end
