function [ptsA,ptsB,normal,JA,JB,dJA,dJB] = pairwiseContactTest(obj,kinsol,body_indA,body_indB,body_collision_indA)

% Uses bullet to perform collision detection between rigid body A and rigid body B
%
% @param kinsol  the output from doKinematics()
% @param body_indA  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param body_indB  numerical index of rigid body B (or the rigidbody object)
%     if body_indB is -1, compute collisions with entire world
% @param body_collision_indA  index vector of collision objects on body A
% 
% @retval ptsA the points (in world coordinates) on bodyA that are in
% collision
% @retval ptsB the point (in world coordinates) on bodyB that are in
% collision
% @retval normal the contact normal vectors on body B in world coordinates
% @retval JA the jacobian of ptsA
% @retval JB the jacobian of ptsB
% @ingroup Collision

if ~isstruct(kinsol)  
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol,nargout>5);
end

if (kinsol.mex ~= true) 
  error('need to call doKinematics using mex first');
end

if (isa(body_indA,'RigidBody')) body_indA = find(obj.body==body_indA,1); end
if (isa(body_indB,'RigidBody')) body_indB = find(obj.body==body_indB,1); end

if nargin > 4
  if body_indB == -1
    [ptsA,ptsB,normal] = collisionmex(obj.mex_model_ptr,3,body_indA,body_collision_indA);
  else
    [ptsA,ptsB,normal] = collisionmex(obj.mex_model_ptr,2,body_indA,body_indB,body_collision_indA);
  end
else
  [ptsA,ptsB,normal] = collisionmex(obj.mex_model_ptr,1,body_indA,body_indB);
end

if isempty(ptsA)
  JA=[]; JB=[];
  return;
end

if (nargout>3)
  x = bodyKin(obj,kinsol,body_indA,ptsA);
  if (nargout>5)
    [~,JA,dJA] = forwardKin(obj,kinsol,body_indA,x);
  else
    [~,JA] = forwardKin(obj,kinsol,body_indA,x);
  end
end
if (nargout>4)
  x = bodyKin(obj,kinsol,body_indB,ptsB);
  if (nargout>6)
    [~,JB,dJB] = forwardKin(obj,kinsol,body_indB,x);
  else
    [~,JB] = forwardKin(obj,kinsol,body_indB,x);
  end
end
