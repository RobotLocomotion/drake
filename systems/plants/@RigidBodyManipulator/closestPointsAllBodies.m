function [ptsA,ptsB,normal,distance,idxA,idxB,JA,JB] = closestPointsAllBodies(obj,kinsol)

% Uses bullet to find the points of closest approach between ALL PAIRS of
% bodies in the manipulator, with the following exceptions:
%   - any body and it's parent in the kinematic tree
%   - body A and body B, where body A belongs to collision filter groups that
%   ignore all of the collision filter groups to which body B belongs
%
% @param kinsol  the output from doKinematics()
%
% @retval ptsA the i-th column contains the point (in body coordinates) 
%   on body idxA(i) (body A) that is closest to body idxB(i) (body B).
% @retval ptsB the i-th column contains the point (in body coordinates) 
%   on body idxB(i) (body B) that is closest to body idxA(i) (body A).
% @retval normal the i-th column contains the contact normal vectors on body
%   idxB(i) in world coordinates
% @retval distance the i-th column contains the signed distance between ptsA(i)
%   and ptsB(i). distance(i) is negative if there is penetration between the
%   two bodies, zero if the bodies are in contact, and positive if they are
%   separated.
% @retval JA the kinematic jacobian of ptsA
% @retval JB the kinematic jacobian of ptsB
% @retval Jd the kinematic jacobian of distance
% @retval idxA link indices for the first body in each checked pair
% @retval idxB link indices for the second body in each checked pair
% @ingroup Collision

if ~isstruct(kinsol)  
  % treat input as closestPointsAllBodies(obj,q)
  kinsol = doKinematics(obj,kinsol,nargout>5);
end

if (kinsol.mex ~= true) 
  doKinematics(obj,kinsol.q);
  warning('RigidBodyManipulator:closestPointsAllBodies:doKinematicsMex', ...
    'Calling doKinematics using mex before proceeding');
end

[ptsA,ptsB,normal,distance,idxA,idxB] = collisionmex(obj.mex_model_ptr);

if nargout > 6
  error('RigidBodyManipulator:closestPointsAllBodies:Jacobians', ...
    'Jacobians temporarily disabled.');
end
