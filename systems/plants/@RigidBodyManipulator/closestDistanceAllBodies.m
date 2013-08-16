function [phi,Jphi] = closestDistanceAllBodies(obj,kinsol)

% Uses bullet to find the minimum distance between ALL PAIRS of bodies in the
% manipulator, with the following exceptions:
%   - any body and it's parent in the kinematic tree
%   - body A and body B, where body A belongs to collision filter groups that
%   ignore all of the collision filter groups to which body B belongs
%
% @param kinsol  the output from doKinematics()
%
% @retval phi minimum distance between each pair of bodies
% @retval Jphi the kinematic jacobian of phi

if (nargout>1)
  [~,~,~,distance,~,~,Jphi] = closestPointsAllBodies(obj,kinsol);
else
  [~,~,~,distance] = closestPointsAllBodies(obj,kinsol);
end

if isempty(distance)
  error('RigidBodyManipulator:closestDistanceAllBodies','''distance'' should not be empty');
else
  phi = distance';
end

end

