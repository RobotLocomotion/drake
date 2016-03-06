function is_in_collision = collidingPointsCheckOnly(obj, kinsol, points, collision_threshold)
% is_in_collision = collidingPoints(obj, kinsol, points, collision_threshold)
% returns true if any point in points is in collision with the manipulator
%
% @param obj
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.  Can also be q, and we'll call doKinematics for you.
% @param points (3 x p) 3D coordinates of p points to check
% @param collision_threshold min distance allowed to consider the point not
% in collision
% @retval bollean indicating whether any point in points is in collision with the manipulator

if ~isstruct(kinsol)
  % treat input as collidingPointsCheckOnly(obj,q)
  kinsol = doKinematics(obj,kinsol);
elseif kinsol.mex == 0
  % doKinematics with mex
  kinsol = doKinematics(obj,kinsol.q);
end

is_in_collision = collidingPointsCheckOnlymex(obj.mex_model_ptr, kinsol.mex_ptr, points, collision_threshold);

end

