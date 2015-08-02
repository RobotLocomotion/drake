function [ptA,ptB,normal,distance,JA,JB,dJA,dJB] = pairwiseClosestPoints(obj,kinsol,bodyA_idx,bodyB_idx)

% Uses bullet to find the points of closest approach between bodies A and B
%
% @param kinsol  the output from doKinematics()
% @param bodyA_idx  numerical index of rigid body A or
%     (less efficient:) a rigidbody object
% @param bodyB_idx  numerical index of rigid body B (or the rigidbody object)
%     if bodyB_idx is -1, compute collisions with entire world
%
% @retval ptA the points (in world coordinates) on bodyA that are in
% collision
% @retval ptB the point (in world coordinates) on bodyB that are in
% collision
% @retval normal the contact normal vectors on body B in world coordinates
% @retval JA the jacobian of ptA
% @retval JB the jacobian of ptB
% @ingroup Collision

if ~isstruct(kinsol)
  % treat input as q
  kinsol = doKinematics(obj,kinsol,compute_second_derivative);
end

active_collision_options.body_idx = [bodyA_idx; bodyB_idx];
[distance,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol,false,active_collision_options);

if isempty(distance)
  ptA = [];
  ptB = [];
  JA =[];
  JB = [];
  dJA = [];
  dJB = [];
else
  if idxA ~= bodyA_idx,
    % then everything is swapped
    normal = -normal;
    tmp = xA;
    xA = xB;
    xB = tmp;
  end
  if nargout > 6,
    [ptA,JA,dJA] = forwardKin(obj,kinsol,bodyA_idx,xA);
    [ptB,JB,dJB] = forwardKin(obj,kinsol,bodyB_idx,xB);
  elseif nargout > 4,
    [ptA,JA] = forwardKin(obj,kinsol,bodyA_idx,xA);
    [ptB,JB] = forwardKin(obj,kinsol,bodyB_idx,xB);
  else
    ptA = forwardKin(obj,kinsol,bodyA_idx,xA);
    ptB = forwardKin(obj,kinsol,bodyB_idx,xB);
  end
end
end
