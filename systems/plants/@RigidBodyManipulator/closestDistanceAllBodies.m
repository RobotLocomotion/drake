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

[ptsA,ptsB,normal,distance,idxA,idxB] = closestPointsAllBodies(obj,kinsol);

n_pairs = numel(idxA);

if isempty(distance)
  error('RigidBodyManipulator:closestDistanceAllBodies','''distance'' should not be empty');
end

%if nargout > 1
  phi = zeros(size(distance'))*kinsol.q(1);

  Jphi = zeros(n_pairs,obj.getNumDOF());
  for i = 1:n_pairs
    [ptA_in_world,JA] = forwardKin(obj,kinsol,idxA(i),ptsA(:,i),0);
    [ptB_in_world,JB] = forwardKin(obj,kinsol,idxB(i),ptsB(:,i),0);
    bodyB_origin = forwardKin(obj,kinsol,idxB(i),zeros(3,1),0);
    normal_in_world = normal(:,i);
    %phi(i) = norm(ptA_in_world-ptB_in_world);
    phi(i) = normal_in_world'*(ptA_in_world-ptB_in_world);
    Jphi(i,:) = normal_in_world'*(JA-JB);
  end
%end

end

