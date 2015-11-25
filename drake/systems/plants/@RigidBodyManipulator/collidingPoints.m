function idx = collidingPoints(obj, kinsol, points, collision_threshold)
% idx = collidingPoints(obj, kinsol, points, collision_threshold)
% returns the indices of the points in collision with any element in the
% manipulator
%
% @param obj
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.  Can also be q, and we'll call doKinematics for you.
% @param points (3 x p) 3D coordinates of p points to check
% @param collision_threshold min distance allowed to consider the point not
% in collision
% @retval indices of the points in collision with any element in the
% manipulator

if ~isstruct(kinsol)
  % treat input as collidingPoints(obj,q)
  kinsol = doKinematics(obj,kinsol);
elseif kinsol.mex == 0
  % doKinematics with mex
  kinsol = doKinematics(obj,kinsol.q);
end

idx = collidingPointsmex(obj.mex_model_ptr, kinsol.mex_ptr, points, collision_threshold);

end

